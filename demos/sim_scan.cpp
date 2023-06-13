#include <iostream>
#include <filesystem>

#include "ForgeScan/TSDF/grid.h"
#include "ForgeScan/TSDF/traversal.h"
#include "ForgeScan/DepthSensor/camera.h"
#include "ForgeScan/DepthSensor/laser.h"
#include "ForgeScan/Primitives/sphere.h"
#include "ForgeScan/Primitives/box.h"
#include "ForgeScan/Utilities/arg_parser.h"


static const Eigen::Vector3d WORLD_ORIGIN(0, 0, 0);


int main(int argc, char* argv[])
{
    ForgeScan::Utilities::ArgParser parser(argc, argv);
    /// Number of pixel in the x and y directions for the depth sensor->
    int nx = 100, ny = 100;

    /// Number of views collected.
    int nv = 10;

    /// Radius for the camera positions.
    double cr = 2.5;

    /// File name for writing XDMF of results.
    std::string fname = "sim_scan";

    /// Create reference to the share directory and verify it exists, creating it if needed.
    std::filesystem::path fpath(FORGESCAN_SHARE_PARAVIEW_DIR);
    fpath.make_preferred();
    std::filesystem::create_directories(fpath);

    /// If true will place the first view in a pre-determined position.
    bool first   = parser.cmdOptionExists("--first");

    /// If true will user randomly selected poses for each view .
    bool random = parser.cmdOptionExists("--random");

    bool laser = parser.cmdOptionExists("--laser");

    {   /// Command line parsing for reading specific arguments
        const std::string &s_nx = parser.getCmdOption("-nx");
        if (!s_nx.empty()) nx = std::stoi(s_nx);

        const std::string &s_ny = parser.getCmdOption("-ny");
        if (!s_ny.empty()) ny = std::stoi(s_ny);

        const std::string &s_num_view = parser.getCmdOption("-nv");
        if (!s_num_view.empty()) nv = std::stoi(s_num_view);

        const std::string &s_rc = parser.getCmdOption("-rc");
        if (!s_rc.empty()) cr = std::stod(s_rc);

        const std::string &s_f = parser.getCmdOption("-f");
        if (!s_f.empty()) fname = s_f;
    }

    /// Create the full file path.
    fpath /= fname;

    std::string sensor_type = laser ? "laser" : "camera";
    std::cout << "Running for " << nv << " " << sensor_type << " sensors with (" << nx << ", " << ny << ") images." << std::endl;
    if (first) {
        std::cout << "Added deterministic first view." << std::endl;
        --nv;  // Decrement the number of requested views for the rest of the program
    }
    std::string strategy = random ? "random" : "uniform";
    std::cout << "Adding " << nv << " views. Using a " + strategy + " strategy" << std::endl;

    // Set up the Grid as a 2m x 2m x 2m cube with 0.02 m resolution
    ForgeScan::TSDF::Grid::Properties properties(0.02, ForgeScan::Vector3ui(101, 101, 101));
    ForgeScan::TSDF::Grid grid(properties);
    grid.translate(ForgeScan::translation(-1, -1, -1));

    ForgeScan::Primitives::Sphere sphere(0.45);
    ForgeScan::Primitives::Box box(1.3, 0.5, 0.5);

    ForgeScan::Primitives::Scene scene{&sphere, &box};

    ForgeScan::DepthSensor::Sensor* sensor = NULL;
    if (laser) {
        ForgeScan::DepthSensor::Intrinsics::Laser sensor_intr(nx, ny, 0, 10, -0.4 * M_PI, 0.4 * M_PI, -0.4 * M_PI, 0.4 * M_PI);
        sensor = new ForgeScan::DepthSensor::Laser(sensor_intr);
    } else {
        ForgeScan::DepthSensor::Intrinsics::Camera sensor_intr(nx, ny, 0, 10, 0.4*M_PI, 0.4*M_PI);
        sensor = new ForgeScan::DepthSensor::Camera(sensor_intr);
    }


    if (first)
    {
        sensor->translate(ForgeScan::point(0, 0, cr));
        sensor->orientPrincipleAxis(WORLD_ORIGIN);

        sensor->image(scene);
        ForgeScan::TSDF::addSensorTSDF(grid, *sensor);
    }


    for (int i = 0; i < nv; ++i)
    {
        if (random) {
            sensor->setPoseRandom(WORLD_ORIGIN, cr);
        } else {
            sensor->setPoseUniform(WORLD_ORIGIN, cr, i, nv);
        }
        sensor->image(scene);
        ForgeScan::TSDF::addSensorTSDF(grid, *sensor);
    }

    grid.saveXDMF(fpath);
    std::cout << "Complete! Saved file in ./share/ParaView as: " + fname << std::endl;

    delete sensor;
    return EXIT_SUCCESS;
}
