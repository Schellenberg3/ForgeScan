#include <iostream>
#include <string>

#include <ForgeScan/voxel_grid.h>

#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>


/// @brief Uses OctoMap tools to load the specified file into the provided Pointcloud
/// @param pc Destination point cloud for the file's data
/// @param fname File name to load data from
/// @note  This only works for OctoMap's custom format. The library also provides tools to
///        write out a .VRML file, but no tools to load it again. This will fail on a .VRML file.
void load_octomap_pc(octomap::Pointcloud &pc, const std::string &fname);


/// @brief   Loads an OctoMap pointcloud from a saved file and places this inside of a VoxelGrid class.
///          This class is then saved to disk in a .csv format.
/// @details The point cloud is added as if a laser sensor at the origin has measured every point. The
///          point cloud must be with in grids bounds of [-1,-1,-1] amd [+1,+1.+1].
/// @todo    This should be updated to use PointCloud library format. Linking issues while writing
///          prevented this initially but it is worth revisiting.
int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cout << "Please give one input file name and one output file name." << std::endl;
        return 1;
    }
    std::string fin_name = argv[1];
    std::string fout_name = argv[2];

    octomap::Pointcloud sphere_pc;
    std::cout << "Loading pointcloud " << fin_name << std::endl;

    load_octomap_pc(sphere_pc, argv[1]);
    std::cout << "Loaded sphere! Found " << sphere_pc.size() << " points!" << std::endl;


    Eigen::Vector3d lower, upper;
    Vector3ui space;
    lower << -1.0, -1.0, -1.0;
    upper << 1.0, 1.0, 1.0;
    space << 100, 100, 100;
    VoxelGrid grid(space, lower, upper);
    std::cout << "Initialized the VoxelGrid" << std::endl;


    Eigen::Vector3d point_xyz, origin(0,0,0);
    int count = 0;
    int fail_count = 0;
    int result = 0;
    for (auto point = sphere_pc.begin(); point != sphere_pc.end(); ++point)
    {
        point_xyz[0] = point->x();
        point_xyz[1] = point->y();
        point_xyz[2] = point->z();

        result = grid.add_point(point_xyz, 1);

        // Adding 100 points should result in each voxel having ~2 points in it
        // notable exceptions are the origin and its neighboring voxels
        grid.add_linear_space(origin, point_xyz, 100);

        fail_count += result;
        count += 1;
    }
    // Re-set origin point to 1 for visualization
    grid.add_point(origin, 1);


    grid.save_csv(fout_name);
    std::cout << "Wrote " << count <<" points with " << fail_count << " out of bounds fails."  << std::endl;
}


void load_octomap_pc(octomap::Pointcloud &pc, const std::string &fname)
{
    std::string ext = fname.substr(fname.size() - 5);
    if (ext == ".vrml")
        throw std::invalid_argument("Do not use .vrml files");

    std::ifstream s(fname.c_str());
    if (!s) 
    {
        std::cout <<"ERROR: could not read " << fname << std::endl;
        return;
    }
    pc.readBinary(s);
}
