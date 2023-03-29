#include <iostream>
#include <string>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/grid_processor.h>
#include <ForgeScan/grid_traversal.h>


/// @brief Simple script for manually adding points to a VoxelGrid within [-1,-1,-1] and [+1,+1,+1].
/// @details  Demonstrates the VoxelGrid `*.HDF5` format and ability to add linearly spaced points.
int main(int argc, char** argv)
{ 
    Eigen::Vector3d lower, upper;
    lower << -1.0, -1.0, -1.0;  
    upper << 1.0, 1.0, 1.0;

    double res = 0.02;

    VoxelUpdate update(1);

    // 2m x 2m x 2m cube with 0.02 m resolution
    VoxelGrid grid(res, lower, upper, false);
    std::cout << "Initialized the VoxelGrid!" << std::endl;

    GridProcessor processor(grid);

    double x, y, z, q = 0;
    Eigen::Vector3d xyz;
    do
    {
        std::cout << "enter the next point in cartesian space: ";
        std::cin >> x >> y >> z >> q;

        xyz[0] = x;
        xyz[1] = y;
        xyz[2] = z;

        grid.set(xyz, update);

    } while (q == 0);

    Eigen::Vector3d start, end;
    start << 0, 0, 1.5;
    end << 0, -1.5, 0;

    int n_ops = 0;
    if (false)
    {
        do {
            std::cout << "Enter 1 to dilate, 2 to erode, or 0 to continue: ";
            std::cin >> q;

            if (q == 1)
            {
                std::cout << "Performing strong  dilation n=1...";
                processor.dilate(1);
            }
            else if (q == 2)
            {
                std::cout << "Performing soft erosion n=5...";
                processor.erode(5);
            }
            
            if (q != 0)
            {
                ++n_ops;
                std::cout << " operation " << n_ops << " finished," << std::endl;
            }

        } while (q != 0);
    }

    std::cout << "Saving to CSV..."  << std::endl;
    grid.save_csv("test_points.csv");

    std::cout << "Exiting program."  << std::endl;
    return 0;
}
