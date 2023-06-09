#include <filesystem>
#include <stdexcept>
#include <iostream>
#include <string>
#include <vector>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/grid_processor.h>


using namespace ForgeScan;

/// @brief Tests initializing, setting, validating, and copying the VoxelGridProperties class.
void testVoxelGridProperties();

/// @brief Tests adding specific locations around the boundaries of a voxel grid. Checks that points were placed in
///        the expected voxels.
void testVoxelGridLocations(ForgeScan::VoxelGrid& grid);

/// @brief Simple script for manually adding points to a VoxelGrid within [-1,-1,-1] and [+1,+1,+1].
/// @details  Demonstrates the VoxelGrid `*.HDF5` format and ability to add linearly spaced points.
int main(int argc, char** argv)
{
    testVoxelGridProperties();

    /// Create our testing VoxelGrid object
    VoxelGrid::Properties properties(0.02, Vector3ui(101, 101, 101));
    VoxelGrid grid(properties);
    grid.translate( translation(-1, -1, -1) );

    testVoxelGridLocations(grid);

    Voxel::Update update(1);

    std::cout << "Initialized the VoxelGrid!" << std::endl;

    GridProcessor processor(grid);

    double x, y, z, q = 0;
    Vector3d xyz;
    Vector3ui grid_index;
    if (false) {
        do
        {
            std::cout << "enter the next point in cartesian space: ";
            std::cin >> x >> y >> z >> q;

            xyz[0] = x;
            xyz[1] = y;
            xyz[2] = z;

            /// Transform xyz to the grid frame and then convert to grid indicies.
            grid.fromWorldToThis(xyz);
            grid_index = grid.pointToGrid(xyz);
            grid.at(grid.pointToGrid(xyz)).update(update);
        } while (q == 0);
    }

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
                // processor.dilate(1);
            }
            else if (q == 2)
            {
                std::cout << "Performing soft erosion n=5...";
                // processor.erode(5);
            }

            if (q != 0)
            {
                ++n_ops;
                std::cout << " operation " << n_ops << " finished," << std::endl;
            }

        } while (q != 0);
    }

    /// Create paths to each share directory.
    std::filesystem::path forgescan_share(FORGESCAN_SHARE_DIR);
    std::filesystem::path paraview_share(FORGESCAN_SHARE_PARAVIEW_DIR);

    /// Ensure that each path matches the system preferred format. And create it if needed.
    forgescan_share.make_preferred();

    paraview_share.make_preferred();
    std::filesystem::create_directories(paraview_share);

    auto hdf5_path = forgescan_share /= "test_points_hdf5";
    std::cout << "Saving to HDF5..."  << std::endl;
    grid.saveHDF5(hdf5_path);

    std::cout << "Loading from HDF5..."  << std::endl;
    VoxelGrid new_grid = loadVoxelGridHDF5(hdf5_path);

    auto xdmf_path = paraview_share /= "test_points";
    std::cout << "Saving to XDMF..."  << std::endl;
    grid.saveXDMF(xdmf_path);

    std::cout << "Exiting program."  << std::endl;
    return 0;
}

void testVoxelGridProperties()
{
    /// Default is ok.
    VoxelGrid::Properties p0;
    p0.isValid();

    /// Copy of an ok one is ok.
    VoxelGrid::Properties p1(p0);

    /// resolution and grid_size is okay.
    VoxelGrid::Properties p2(0.1, Vector3ui(18, 19, 20));

    /// grid_size and resolution is okay.
    VoxelGrid::Properties p3(Vector3ui(18, 19, 20), Vector3d(1, 1.5, 2));

    /// The setups I typically use:
    VoxelGrid::Properties p4(Vector3ui(101, 101, 101), Vector3d(2, 2, 2));
    VoxelGrid::Properties p5(0.02, Vector3ui(101, 101, 101));

    /// Test failing on resolution
    VoxelGrid::Properties p_res1, p_res2;
    p_res1.resolution = -1;
    try { p_res1.isValid(); } catch (const std::invalid_argument& e) {
        std::cout << "\n[Resolution] Should error on non-positive resolution:\n" << e.what() << std::endl;
    }
    p_res2.resolution *= 2;
    try { p_res2.isValid(); } catch (const std::invalid_argument& e) {
        std::cout << "\n[Resolution] Should error on dimension check:\n" << e.what() << std::endl;
    }

    /// Test failing on min/max
    VoxelGrid::Properties p_mm1, p_mm2;
    p_mm1.min_dist =  100;
    p_mm2.max_dist = -100;
    try { p_mm1.isValid(); } catch (const std::invalid_argument& e) {
        std::cout << "\n[Min/Max] Should error on min/max check:\n" << e.what() << std::endl;
    }
    try { p_mm2.isValid(); } catch (const std::invalid_argument& e) {
        std::cout << "\n[Min/Max] Should error on min/max check:\n" << e.what() << std::endl;
    }

    /// Test failing on dimensions less than or equal to zero
    VoxelGrid::Properties p_dims;
    p_dims.dimensions[0] = -10;
    try { p_dims.isValid(); } catch (const std::invalid_argument& e) {
        std::cout << "\n[Dimensions] Should error on dimension greater than zero check:\n" << e.what() << std::endl;
    }

    /// Test failing on grid_size less than or equal to zero
    VoxelGrid::Properties p_gs;
    p_gs.grid_size[0] = 0;
    try { p_gs.isValid(); } catch (const std::invalid_argument& e) {
        std::cout << "\n[GridSize] Should error on grid_size greater than zero check:\n" << e.what() << std::endl;
    }

    /// Test failing on copy of an invalid VoxelGrid::Properties
    VoxelGrid::Properties p_invalid;
    p_invalid.dimensions.array() *= -1;
    try { VoxelGrid::Properties p_copy(p_invalid); } catch (const std::invalid_argument& e) {
        std::cout << "\n[Copy::Dimensions] Should error on dimensions greater than zero check:\n" << e.what() << std::endl;
    }
    VoxelGrid::Properties p_copy;
    p_invalid.resolution *= -1;
    try { p_copy = p_invalid; } catch (const std::invalid_argument& e) {
        std::cout << "\n[Copy::Resolution] Should error on resolution non-positive check:\n" << e.what() << std::endl;
    }
}


void testVoxelGridLocations(ForgeScan::VoxelGrid& grid)
{
    /// Maximum distance the point should be moved when it is rounded to the nearest voxel.
    const double max_expected_distance = (std::sqrt(3) * 0.5 * grid.properties.resolution) * 1.001;
    double distance = 0;

    std::vector<point> inputs {
        point(0, 0, 0),                // [50, 50, 50]
        point(-1, -1, -1),             // [0, 0, 0]
        point(-1.009, -1.009, -1.009), // [0, 0, 0]
        point(-0.991, -0.991, -0.991), // [0, 0, 0]
        point(-0.990, -0.990, -0.990), // [0, 0, 0]
        point(-0.989, -0.989, -0.989), // [1, 1, 1]
        point(-0.980, -0.980, -0.980), // [1, 1, 1]
        point(-0.979, -0.979, -0.979), // [1, 1, 1]
        point(1, 1, 1),          // [100, 100, 100]
        point(1.01, 1.01, 1.01), // [100, 100, 100]
        point(0.99, 0.99, 0.99), // [100, 100, 100]
    };
    grid_idx c_idx(0, 0, 0);
    point  c_point(0, 0, 0);

    /// The point in the grid's reference frame.
    point  p_grid(0, 0, 0);

    int i = 0;
    for (const auto& p : inputs)
    {
        p_grid = grid.fromWorldToThis(p);

        c_idx = grid.pointToGrid(p_grid);
        c_point = grid.gridToPoint(c_idx);

        distance  = (p_grid - c_point).norm();

        std::cout << "\n\nCHECKING " << i
                  << "\n\tInput  = " << p.transpose()
                  << "\n\tInput  = " << p_grid.transpose()
                  << "\n"
                  << "\n\tGrid   = " << c_idx.transpose()
                  << "\n\tCheck  = " << c_point.transpose()\
                  << "\n\tDist   = " << distance
                  << "\n\t\tExpected: " << distance - max_expected_distance
                  << std::endl;
        ++i;
    }
}
