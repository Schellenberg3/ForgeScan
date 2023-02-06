#include <iostream>
#include <fstream>
#include <iomanip>

#include <octomap/octomap.h>


/// @brief Samples occupancy data in a equally divided grid from the given OcTree and saves it as a CSV
///        which increases fastest in X, then Y, then Z.
/// @param tree OcTree to be sampled and saved.
/// @param spacing Number of samples to take in each direction.
/// @note This does NOT sample evenly for two reasons. First, the distance between points is derived from
///       the spacing and the min and max of each directions bounding box. Each direction may span a different
///       distance. Second, the occupancy search is cut off at a depth of 12 (max depth is 16).
void save_octomap_grid_sample(const octomap::OcTree tree, const int spacing = 10);


/// @brief Loads a specified OcTree file (*.bt) and performs grid sampling on it
///        which is then saved to a .csv file named `octomap_voxel_sample.csv`.
int main(int argc, char** argv){
    if (argc != 2) {
        std::cout << "Please give one input file name." << std::endl;
        return 1;
    }

    std::cout << "Looking for tree: " << argv[1] << std::endl;
    octomap::OcTree tree(argv[1]);

    std::cout << "Loaded a tree with " << tree.getNumLeafNodes() << " leaf nodes. Saving voxel csv now." << std::endl;
    save_octomap_grid_sample(tree, 100);

    return 0;
}


void save_octomap_grid_sample(const octomap::OcTree tree, const int spacing){
    // Get the tree size
    double x, y, z, 
           max_x, max_y, max_z, 
           min_x, min_y, min_z,  
           dx, dy, dz;
    double occ;

    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);

    dx = (max_x - min_x) / (spacing - 1);
    dy = (max_y - min_y) / (spacing - 1);
    dz = (max_z - min_z) / (spacing - 1);

    std::cout << "Max, Min, Spacing\n"
              << max_x << ", " << min_x << ", " << dx << "\n"
              << max_y << ", " << min_y << ", " << dy << "\n"
              << max_z << ", " << min_z << ", " << dz << "\n"
              << std::endl;

    // Set up file stream
    std::ofstream fname;
    fname.open("octomap_voxel_sample.csv");
    if (fname.is_open() == false) 
    {
        std::cout << "Could not open file!" << std::endl;
    }

    fname << std::fixed << std::setprecision(8);
    fname << "Test voxel sampling from Octree" << std::endl;
    fname << "occupancy" << std::endl;

    z = min_z;
    for (int i = 0; i < spacing; ++i)
    {          // i -> Increment Z
        y = min_y;
        for (int j = 0; j < spacing; ++j) 
        {      // j -> Increment y
            x = min_x;
            for (int k = 0; k < spacing; ++k) 
            {  // k -> Increment X
                octomap::OcTreeNode* node_ptr = tree.search(x, y, z, 12);

                if (node_ptr != NULL)
                    occ = node_ptr->getOccupancy();
                else
                    occ = -1;

                fname << occ << std::endl;
                x += dz;
            }
            y += dy;
        }
        z += dx;
    }
    fname.close();
}


