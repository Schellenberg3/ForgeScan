#include <filesystem>
#include <iostream>
#include <string>
#include <random>
#include <cmath>

#include <Eigen/Dense>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/grid_processor.h>
#include <ForgeScanUtils/timing_utils.h>
#include <ForgeScanUtils/arg_parser.h>


/// @brief Adds random lines to the grid. Does so with a fixed-seed random number generator.
/// @param grid Grid to add lines to.
/// @param n    Number of lines to add.
/// @param exact Boolean flag, if true will to use exact line else the TSDF line is plotted. Default false.
/// @param seed  Seed for the RNG. Default is 1.
void addPseudoRandomLines(ForgeScan::VoxelGrid& grid, const int& n, const bool& exact = false, const int& seed = 1);

/// @brief Simple script for manually adding points to a VoxelGrid within [-1,-1,-1] and [+1,+1,+1].
/// @details  Demonstrates the VoxelGrid `*.HDF5` format and ability to add linearly spaced points.
int main(int argc, char** argv)
{ 
    ArgParser parser(argc, argv);

    int n = 10;
    std::string n_parse = parser.getCmdOption("-n");
    if ( !n_parse.empty() ) n = std::abs( std::stoi( n_parse ) );

    int seed = 1;
    std::string seed_parse = parser.getCmdOption("-s");
    if ( !seed_parse.empty() ) seed = std::abs( std::stoi( seed_parse ) );

    bool exact = parser.cmdOptionExists("-e");

    /// Set up the VoxelGrid as a 2m x 2m x 2m cube with 0.02 m resolution, positioned at the world origin.
    ForgeScan::VoxelGridProperties properties(0.02, ForgeScan::Vector3ui(101, 101, 101), -0.2, 0.2);
    ForgeScan::VoxelGrid grid_random(properties);

    /// Add upper and lower boundary markers to the random grid.
    grid_random.at(ForgeScan::grid_idx(0, 0, 0)).update(ForgeScan::VoxelUpdate(1, 0, 0, 0));
    grid_random.at(ForgeScan::grid_idx(100, 100, 100)).update(ForgeScan::VoxelUpdate(1, 0, 0, 0));

    addPseudoRandomLines(grid_random, n, exact, seed);

    return 0;
}

void addPseudoRandomLines(ForgeScan::VoxelGrid& grid, const int& n, const bool& exact, const int& seed)
{
    ForgeScan::VoxelUpdate update(1, 0, 0, 0);

    std::vector<Eigen::Vector3d> s_points, e_points;
    s_points.reserve(n);
    e_points.reserve(n);
    {   /// Technically an allocator lambda passed to the vector constructor might be better.
        /// But this explicit method works too...
        std::mt19937 gen(seed);

        const double range = grid.properties.dimensions.maxCoeff();
        const double extra = 0.1 * range;
        std::uniform_real_distribution<> dist(- extra, range + extra);

        for (int i = 0; i < n; ++i) {
            s_points.push_back(Eigen::Vector3d(dist(gen), dist(gen), dist(gen)));
            e_points.push_back(Eigen::Vector3d(dist(gen), dist(gen), dist(gen)));
        }
    }

    std::string method = exact ? "exact" : "TSDF";
    std::cout << "Generated " << n << " random lines to add with the " << method << " method." << std::endl;
    for (int i = 0, max_idx = n > 10 ? 10 : n; i < max_idx; ++i) {
        std::cout << "["<<i<<"] " << s_points[i].transpose() << " -> " << e_points[i].transpose() << std::endl;
    }

    ForgeScan::Utils::SimpleTimer timer;
    timer.start();
    for (int i = 0; i < n; ++i) {
        if (exact)
            grid.addRayExact(update, s_points[i], e_points[i]);
        else
            grid.addRayTSDF(s_points[i], e_points[i]);
    }
    timer.stop();
    std::cout << "Run time in ms was:\n\t" << timer.elapsedMilliseconds() << std::endl;

    std::filesystem::path random_fpath(FORGESCAN_SHARE_PARAVIEW_DIR);
    random_fpath /= "test_traversal_random";
    std::cout << "Saving to disk in XDMF format..."  << std::endl;
    grid.saveXDMF(random_fpath);
}
