#include <iostream>
#include <string>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/grid_processor.h>
#include <ForgeScan/grid_traversal.h>


/// @brief Simple script for manually adding points to a VoxelGrid within [-1,-1,-1] and [+1,+1,+1].
/// @details  Demonstrates the VoxelGrid `*.HDF5` format and ability to add linearly spaced points.
int main(int argc, char** argv)
{ 
    Eigen::Vector3d lower(-1.0, -1.0, -1.0), upper(1.0, 1.0, 1.0);
    double res = 0.02;

    VoxelGrid grid(res, lower, upper, false);  // 2m x 2m x 2m cube with 0.02 m resolution
    std::cout << "Initialized the VoxelGrid!" << std::endl;

    voxel_distance d_ls = 1;
    VoxelElementUpdate update_ls(&d_ls);

    voxel_distance d_la = 10;
    VoxelElementUpdate update_la(&d_la);

    voxel_distance d_le = 20;
    VoxelElementUpdate update_le(&d_le);

    point rs = Vector3d(-0.8, -0.7, -0.6);
    point re = Vector3d(0.1, 0.1, 0.1);

    // Linear Space
    if (true)
    {
        std::cout << "Adding linear spaced ray" << std::endl;
        point rs_ls = rs;
        point re_ls = re;
        addRayLinspace(
            grid, update_ls,
            rs_ls, re_ls,
            60
        );
    }

    // Approx Line
    if (true)
    {
        std::cout << "Adding approximate ray" << std::endl;
        point rs_la = rs;
        point re_la = re;
        for (int i = 0; i < 1; ++i)
        {
            //rs_la[i] += 0.15;
            //re_la[i] += 0.15;
        }
        addRayApprox(
            grid, update_la,
            rs_la, re_la,
            0.9
        );
    }

    // Exact Line
    if (true)
    {
        std::cout << "Adding exact ray" << std::endl;
        point rs_le = rs;
        point re_le = re;
        for (int i = 0; i < 1; ++i)
        {
            //rs_le[i] += 0.3;
            //r_e_le[i] += 0.3;
        }
        addRayExact(
            grid, update_le,
            rs_le, re_le,
            0, 1
        );
    }

    std::cout << "Saving to CSV..."  << std::endl;
    grid.save_csv("test_points.csv");

    std::cout << "Exiting program."  << std::endl;
    return 0;
}