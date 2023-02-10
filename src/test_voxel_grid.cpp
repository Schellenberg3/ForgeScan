#include <iostream>
#include <string>

#include <point_gen/voxel_grid.h>

#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>


/// @brief Simple script for manually adding points to a VoxelGrid within [-1,-1,-1] and [+1,+1,+1].
/// @details  Demonstrates the VoxelGrid `*.HDF5` format and ability to add linearly spaced points.
int main(int argc, char** argv)
{ 
    Eigen::Vector3d lower, upper;
    lower << -1.0, -1.0, -1.0;  
    upper << 1.0, 1.0, 1.0;

    double res = 0.02;

    // 2m x 2m x 2m cube with 0.02 m resolution
    VoxelGrid grid(res, lower, upper);
    std::cout << "Initialized the VoxelGrid" << std::endl;

    double x, y, z, q = 0;
    Eigen::Vector3d xyz;
    do
    {
        std::cout << "enter the next point in cartesian space: ";
        std::cin >> x >> y >> z >> q;

        xyz[0] = x;
        xyz[1] = y;
        xyz[2] = z;

        grid.set(xyz, 1);

    } while (q == 0);

    Eigen::Vector3d start, end;
    start << 0, 0, 1.5;
    end << 0, -1.5, 0;

    grid.add_linear_space(start, end, 40);

    grid.save_csv("test_points.csv");

    grid.save_hdf5("test_points.h5");
    grid.load_hdf5("test_points.h5");

    return 0;
}
