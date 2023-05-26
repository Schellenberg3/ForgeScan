#include <iostream>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/depth_sensor.h>

#include <ForgeScan/Primitives/sphere.h>
#include <ForgeScan/Primitives/box.h>

using namespace ForgeScan;
static const Eigen::Vector3d WORLD_ORIGIN(0, 0, 0);


/// @brief Simulates the scanning of a scene by a depth sensor.
/// @param nx Positive integer number of points to add to the sensor. Default 100.
/// @param ny Positive integer number of points to add to the sensor. Default 100.
/// @param num_view  Positive integer number of views to add. Default 10.
/// @param first_pos If true will set the first view to be fixed down the world +Z axis. Default `true`.
/// @param random_pose If included will randomly sample new poses. Be default poses are evenly distributed.
int main(int argc, char* argv[])
{
    int nx = 100, ny = 100, num_view = 10;
    bool first_pos = true, random_pose = false;
    double camera_radius = 2.5;
    if (argc >= 2) nx  = std::abs( std::stoi(argv[1]) );  // Set the number of sensor points
    if (argc >= 3) ny  = std::abs( std::stoi(argv[2]) );  // Set the number of sensor points
    if (argc >= 4) num_view = std::abs( std::stoi(argv[3]) );  // Set the number of sensors
    if (argc >= 5) first_pos = (argv[4] == "true");            // If true, sets first view to be fixed down the world +Z axis
    if (argc >= 6) random_pose = !random_pose;                 // If anything is passed for argv[4], we will use a random sampling strategy
    std::cout << "Running for " << num_view << " sensors with (" << nx << ", " << ny << ") images." << std::endl;

    // Set up the VoxelGrid as a 2m x 2m x 2m cube with 0.02 m resolution
    ForgeScan::VoxelGridProperties props(0.2);
    props.dimensions = ForgeScan::Vector3d(2, 2, 2);
    props.grid_size = ForgeScan::Vector3ui(100, 100, 100);
    props.resolution = -1;  // Let the grid size and dimensions set the resolution.
    ForgeScan::translation move(-1, -1, -1);

    ForgeScan::VoxelGrid grid(props, move);

    ForgeScan::Primitives::Sphere sphere(0.45);
    ForgeScan::Primitives::Box box(1.3, 0.5, 0.5);

    ForgeScan::Primitives::Scene scene{&sphere, &box};

    ForgeScan::Intrinsics::DepthCamera sensor_intr(nx, ny, 0, 10, 0.4*M_PI, 0.4*M_PI);
    // ForgeScan::Intrinsics::LaserScanner sensor_intr(nx, ny, 0, 10, -0.1 * M_PI, 0.1 * M_PI, -0.1 * M_PI, 0.1 * M_PI);

    ForgeScan::DepthSensor::DepthCamera sensor(sensor_intr);
    // ForgeScan::DepthSensor::LaserScanner sensor(sensor_intr);

    if (first_pos)
    {
        sensor.translate(point(0, 0, camera_radius));
        sensor.orientPrincipleAxis(WORLD_ORIGIN);

        sensor.image(scene);
        grid.addSensor(sensor);

        std::cout << "Added deterministic first view." << std::endl;
        --num_view;  // Decrement the number of requested views for the rest of the program
    }

    std::string strategy = random_pose ? "random" : "uniform";
    std::cout << "Adding " << num_view << " new views. Using a " + strategy + " strategy" << std::endl;
    for (int i = 0; i < num_view; ++i)
    {
        if (random_pose) {
            sensor.setPoseRandom(WORLD_ORIGIN, camera_radius);
        } else {
            sensor.setPoseUniform(WORLD_ORIGIN, camera_radius, i, num_view);
        }
        sensor.image(scene);
        grid.addSensor(sensor);
    }
    std::string fname = "sim_sphere_scan";
    grid.saveXDMF(fname);
    std::cout << "Complete! Saved file as: " + fname << std::endl;
    return 0;
}
