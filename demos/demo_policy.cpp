#include <iostream>
#include <filesystem>

#include "ForgeScan/Policies/random.h"
#include "ForgeScan/Policies/uniform.h"
#include "ForgeScan/Policies/max_discrepancy.h"
#include "ForgeScan/Policies/low_discrepancy.h"

#include "ForgeScan/DepthSensor/camera.h"
#include "ForgeScan/DepthSensor/laser.h"
#include "ForgeScan/Primitives/sphere.h"
#include "ForgeScan/Primitives/box.h"
#include "ForgeScan/Primitives/scene.h"

#include "ForgeScan/Utilities/arg_parser.h"


int main(int argc, char* argv[])
{
    ForgeScan::Utilities::ArgParser parser(argc, argv);
    /// Number of pixel in the x and y directions for the depth sensor->
    int nx = 100, ny = 100;

    /// Number of views collected.
    int nv = 10;

    /// Radius for the camera positions.
    double cr = 2.5;

    /// Seed for the policy. Negative one will use a random seed in each policy.
    int seed = -1;

    /// File name for writing XDMF of results.
    std::string fname = "demo_policy";

    /// Create reference to the share directory and verify it exists, creating it if needed.
    std::filesystem::path fpath(FORGESCAN_SHARE_PARAVIEW_DIR);
    fpath.make_preferred();
    std::filesystem::create_directories(fpath);

    /// If true will place the first view in a pre-determined position.
    bool first = parser.cmdOptionExists("--first");
    bool laser = parser.cmdOptionExists("--laser");

    /// Default for no passed value:
    ForgeScan::Policies::Type policy_type = ForgeScan::Policies::Type::UniformSphereRandom;

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

        const std::string &s_s = parser.getCmdOption("-s");
        if (!s_s.empty()) seed = std::stoi(s_s);

        const std::string &s_p = parser.getCmdOption("-p");
        if (!s_p.empty()) {
            if (s_p == "uso") {
                policy_type = ForgeScan::Policies::Type::UniformSphereOrdered;
            } else if (s_p == "usr") {
                policy_type = ForgeScan::Policies::Type::UniformSphereRandom;
            } else if (s_p == "rs") {
                policy_type = ForgeScan::Policies::Type::RandomSphere;
            } else if (s_p == "mds") {
                policy_type = ForgeScan::Policies::Type::MaxDiscrepancySphere;
            }else if (s_p == "mdsri") {
                policy_type = ForgeScan::Policies::Type::MaxDiscrepancySphereRandomInit;
            } else {
                std::cerr << "[Demos::demo_policy::main] Invalid policy flag. Exiting program." << std::endl;
                return EXIT_FAILURE;
            }
        }
    }

    /// Create the full file path.
    fpath /= fname;

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

    ForgeScan::Policies::Policy* policy = NULL;
    switch (policy_type) {
        case ForgeScan::Policies::Type::UniformSphereOrdered:
            policy = new ForgeScan::Policies::UniformSphereOrdered(grid, *sensor, scene, nv, cr);
            break;
        case ForgeScan::Policies::Type::UniformSphereRandom:
            policy = new ForgeScan::Policies::UniformSphereRandom(grid, *sensor, scene, nv, cr, seed);
            break;
        case ForgeScan::Policies::Type::MaxDiscrepancySphere:
            policy = new ForgeScan::Policies::MaxDiscrepancySphere(grid, *sensor, scene, nv, cr, seed);
            break;
        case ForgeScan::Policies::Type::MaxDiscrepancySphereRandomInit:
            policy = new ForgeScan::Policies::MaxDiscrepancySphereRandomInit(grid, *sensor, scene, nv, cr, 3, seed);
            break;
        default:
            policy = new ForgeScan::Policies::RandomSphere(grid, *sensor, scene, nv, cr, seed);
            break;
    }

    std::cout << "Adding " << nv << " views. Using a " + policy->getName() + " policy" << std::endl;

    policy->run();
    policy->save(fpath);

    std::cout << "Complete! Saved policy results in ./share/ParaView as: " + fname << std::endl;

    delete sensor;
    delete policy;

    return EXIT_SUCCESS;
}
