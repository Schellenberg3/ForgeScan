#include <iostream>
#include <string>
#include <random>

#include <Eigen/Dense>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/grid_processor.h>
#include <ForgeScan/grid_traversal.h>
#include <ForgeScan/timing_utils.h>


/// @brief Simple script for manually adding points to a VoxelGrid within [-1,-1,-1] and [+1,+1,+1].
/// @details  Demonstrates the VoxelGrid `*.HDF5` format and ability to add linearly spaced points.
int main(int argc, char** argv)
{ 
    // 2m x 2m x 2m cube with 0.02 m resolution
    VoxelGridProperties props(0.2);
    props.dimensions = Vector3d(2, 2, 2);
    props.grid_size = Vector3ui(100, 100, 100);
    props.resolution = -1;  // Let the grid size and dimensions set the resolution.
    translation move(-1, -1, -1);

    VoxelGrid grid_exact(props, move);

    std::cout << "Initialized each VoxelGrid!" << std::endl;

    VoxelUpdate update(1, 0, 0, 0);

    int num = argc > 1 ? std::stoi(argv[1]) : (1280 * 720) ; // Defualt to RealSense D455 resolution
    std::vector<Vector3d> start_vecs, end_vecs;
    start_vecs.reserve(num);
    end_vecs.reserve(num);
    {
        // std::random_device rd; // Call rd() for a random seed while initializing the generator.
        std::mt19937 gen(1);
        std::uniform_real_distribution<> dist(-1.9, 1.9);
        
        for (int i = 0; i < num; ++i)
        {
            start_vecs.push_back(Vector3d(dist(gen), dist(gen), dist(gen)));
            end_vecs.push_back(Vector3d(dist(gen), dist(gen), dist(gen)));
        }

        // Set the first value to the case where the start/end point are the same.
        end_vecs[0] = start_vecs[0];

        // Display the first 10 (or all if the requested number is less than 10)
        for (int i = 0, max_idx = num > 10 ? 10 : num; i < max_idx; ++i)
        {
            std::cout << "["<<i<<"] " << start_vecs[i].transpose() << " -> " << end_vecs[i].transpose() << std::endl; 
        }
    }
    std::cout << "Generated " << num << " random lines to add with each method." << std::endl;

    SimpleTimer t_lin, t_apr, t_exa;


    t_exa.start();
    for (int i = 0; i < num; ++i)
        {
        addRayExact(
            grid_exact, update,
            start_vecs[i], end_vecs[i], 
            0.0, 1.0
        );
    }
    t_exa.stop();
    std::cout << "Added " << num << " with exact method" << std::endl;

    std::cout << "Run time in ms was:\n\tLinear\n\t\t"
              << "\n\tExact method:\n\t\t" << t_exa.elapsedMilliseconds() << std::endl;

    
    if (argc > 2)
    {
        std::cout << "Saving to disk in XDMF format..."  << std::endl;
        grid_exact.saveXDMF("test_exact");
    }
    else {
        std::cout << "Not saving data."  << std::endl;
    }

    std::cout << "Exiting program."  << std::endl;
    return 0;
}