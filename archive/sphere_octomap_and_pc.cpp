#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>

// Removed for the time due to linker issues
// #include <pcl-1.10/pcl/io/pcd_io.h>
// #include <pcl-1.10/pcl/point_cloud.h>
// #include <pcl-1.10/pcl/point_types.h>

#include <iostream>
#include <random>
#include <math.h>


/// @brief Samples the given number of points from a unit sphere and places that data in a OctoMap
///        PointCloud and then inserts that into the OcTree.
/// @param n_pts Number of points to uniformly sample.
/// @param octree Output OcTree.
/// @param save_pc If true will save the Pointcloud before inserting it into the OcTree. 
void sample_sphere(const int& n_pts, octomap::OcTree& octree, const bool& save_pc = true);


/// @brief Generates an OcTree and a Pointcloud with octomap. Both contain points uniformly sampled from
///        a unit sphere. Once generated the OcTree is saved as a *.bt file and the sphere is saved as
///        an octomap formatted Pointcloud (no file extension).
/// @param num_pts Integer number of points to sample from the sphere. Default is 1000.
int main(int argc, char** argv){
    int num_pts = 1000;
    if (argc >= 2) num_pts = std::abs( std::stoi(argv[1]) );

    // Tree with .001 mm resolution
    octomap::OcTree tree(0.001);

    sample_sphere(num_pts, tree);

    double x, y, z;
    tree.getMetricSize(x, y, z);
    std::string dims = "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";

    std::cout << "Generated tree of " << dims << " with " << num_pts << " point and " << tree.getNumLeafNodes() 
              << " leaf nodes." << std::endl;
    
    std::string fname =  "ot_sphere_" + std::to_string(num_pts) + ".bt";
    tree.writeBinary(fname);

    return 0;
}


void sample_sphere(const int& n_pts, octomap::OcTree& octree, const bool& save_pc)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0, 1);

    float max_theta = 2 * M_PI;
    float max_phi = M_PI;
    float x, y, z, theta, phi, sin_phi;

    // Octomap point cloud set-up
    octomap::point3d origin(0, 0, 0);
    octomap::Pointcloud pc;
    pc.reserve(n_pts);

    // PCL Set-up
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // cloud.width = n_pts;
    // cloud.height = 1;  // 1 implies an un-ordered list of points, otherwise it becomes a voxel...
    // cloud.is_dense = false;
    // cloud.resize(cloud.width * cloud.height);


    for (int i = 0; i < n_pts; ++i)
    {
        theta = max_theta * dist(gen);
        phi = std::acos(1 - 2 * dist(gen));

        sin_phi = std::sin(phi);
        y = std::sin(theta) * sin_phi;
        x = std::cos(theta) * sin_phi;
        z = std::cos(phi);

        // Octomap point cloud 
        octomath::Vector3 endpoint(x, y, z);
        pc.push_back(endpoint);
    
        // Point cloud library
        // cloud.at(i).x = x;
        // cloud.at(i).y = y;
        // cloud.at(i).z = z;
    }

    std::cout << "Point cloud of size: " << pc.size() << std::endl;
    if (save_pc)
    {
        // Save octomap PC
        std::string pc_fname = "/vrml/pc_sphere_" + std::to_string(n_pts);
        std::ofstream s(pc_fname.c_str());    
        if (!s) 
        {
            std::cout <<"ERROR: could not read " << pc_fname << std::endl;
            return;
        }
        pc.writeBinary(s);

        std::string pc_vrml_fname = pc_fname + ".vrml";
        pc.writeVrml(pc_vrml_fname);

        // Save point cloud
        // pcl::io::savePCDFile("/pcd/" + pc_fname + ".pcd", cloud, false);
    }

    // This is actually the slow part of the function
    octree.insertPointCloud(pc, origin, true);
}


