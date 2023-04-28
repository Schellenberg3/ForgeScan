#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/grid_traversal.h>

#include <Eigen/Geometry>

#include <iostream>
#include <random>
#include <cmath>





/// @brief Simulates the scanning of a sphere by a narrow FOV laser scanner.
/// @param num_pts   Positive, integer number of points to add to the sensor. Default 1000.
/// @param num_view  Positive, integer number of views to add. Default 10.
/// @param first_pos If true will set the first view to be fixed down the world +Z axis. Default `true`.
/// @param random_pose If included will randomly sample new poses. Be default poses are evenly distributed.
int main(int argc, char* argv[])
{
    int num_pts = 1000, num_view = 10;
    bool first_pos = true, random_pose = false;
    if (argc >= 2) num_pts  = std::abs( std::stoi(argv[1]) );  // Set the number of sensor points
    if (argc >= 3) num_view = std::abs( std::stoi(argv[2]) );  // Set the number of sensors
    if (argc >= 4) first_pos = (argv[3] == "true");            // If true, sets first view to be fixed down the world +Z axis
    if (argc >= 5) random_pose = !random_pose;                 // If anything is passed for argv[4], we will use a random sampling strategy
    std::cout << "Running for " << num_view << " sensors with " << num_pts << " samples each." << std::endl;

    // Set up the VoxelGrid as a 2m x 2m x 2m cube with 0.02 m resolution
    Eigen::Vector3d lower(-1, -1, -1), upper(1, 1, 1);
    VoxelGrid grid(0.02, lower, upper, false);

    double res = 0.02;

    // 2m x 2m x 2m cube with 0.02 m resolution
    VoxelGrid grid(res, lower, upper, false);


    // Set up the simulated sensor
    Eigen::Vector3d position, normal;
    SimSensorReading laser_scan(position, normal, num_pts);
    sample_sphere_to_sensor(&laser_scan);  // Populate the sensor


    // Optional deterministic first view
    // This is positioned along the +Z axis, facing the origin.
    if (first_pos)
    {
        laser_scan.position << 0, 0, SIM_RHO;
        laser_scan.normal << 0, 0, -1;

        addSensor(grid, laser_scan);

        // std::cout << "\nAdding first scanner at: \n" << laser_scan.position.transpose() << std::endl;
        // std::cout << "With normal of: \n" << laser_scan.normal.transpose() << std::endl;

        std::cout << "Added deterministic first view." << std::endl;
        --num_view;  // Decrement the number of requested views for the rest of the program
    }

    std::string strategy = random_pose ? "random" : "uniform";
    std::cout << "Adding " << num_view << " new views. Using a " + strategy + " strategy" << std::endl;
    for (int i = 0; i < num_view; ++i)
    {
        uniform_sample_camera_pose(position, normal, num_view);
        // random_sample_camera_pose(position, normal);
        laser_scan.position << position;
        laser_scan.normal << normal;

        // std::cout << "\nAdding scanner at: \n" << laser_scan.position.transpose() << std::endl;
        // std::cout << "With normal of: \n" << laser_scan.normal.transpose() << std::endl;
        addSensor(grid, laser_scan);
    }
    std::string fname = "sim_sphere_scan";
    grid.saveXDMF(fname);
    std::cout << "Complete! Saved file as: " + fname << std::endl;
    return 0;
}
