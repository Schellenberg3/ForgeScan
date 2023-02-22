#include <iostream>
#include <string>

#include <ForgeScan/voxel_grid.h>


/// @brief   Generates a VoxelGrid which illustrates what a depth camera would look like if it did not intersect
///          any points in the space. 
/// @details This really only exists to demonstrate that lines may be drawn between points.
/// @param fname File name to save the grid's CSV as.
int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Please give one output file name." << std::endl;
        return 1;
    }
    std::string fname = argv[1]; 

    Eigen::Vector3d lower, upper;
    lower << -1.0, -1.0, -1.0;
    upper << 1.0, 1.0, 1.0;
    
    Vector3ui space;
    space << 100, 100, 100;

    VoxelGrid grid(space, lower, upper);
    std::cout << "Initialized the VoxelGrid" << std::endl;


    Eigen::Vector3d point_xyz, origin;
    origin << 0, 0, -1.5;
    point_xyz << 0, 0, 0;

    point_xyz[2] = 0.5;
    for (float x = -0.5; x <= 0.5; x += 0.1)
    {
        for (float y = -0.5; y <= 0.5; y += 0.1)
        {
            point_xyz[0] = x;
            point_xyz[1] = y;
            grid.add_point(point_xyz, 1);

            // Z-distance of 2 so 300 should give the average
            // line about 2 points/voxel with the resolution of 50 voxel/unit
            grid.add_linear_space(origin, point_xyz, 300);
        }
    }
    grid.add_point(origin, 250);
    grid.save_csv(fname);
}
