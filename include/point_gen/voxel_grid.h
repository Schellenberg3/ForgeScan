# ifndef MY_VOXEL_H
# define MY_VOXEL_H

#include <point_gen/sim_sensor_reading.h>

#include <Eigen/Geometry>

#include <vector>
#include <string>


/// Convenience typedef for Eigen
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3i Vector3i;

/// Convenience typedef for Eigen; 32 bit unsigned
typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;


/// @brief Container for a 3 dimensional grid ov voxels.
class VoxelGrid
{
    private:
        /// Vector structure for the voxels. Increments fastest in X, then Y, then Z.
        std::vector<uint8_t> grid;

        /// Spacing in the X, Y, and Z direction
        Vector3ui space;

        /// Lower (X, Y, Z) bound for coordinates inside of grid
        Eigen::Vector3d lower;

        /// Upper (X, Y, Z) bound for coordinates inside of grid
        Vector3d upper;

        /// Pre-computed scaling factor for point placement
        Vector3d idx_scale;


        /// @brief Sets the voxel to the provided value
        /// @param idx The index of the voxel within the vector
        /// @param val The value to insert at the voxel index
        /// @return 0 if the voxel was accessed successfully or 1 if the index could not be accesses
        int set_voxel(const Vector3ui& idx, const uint8_t& val = 1);


        /// @brief Increases the value at the voxel location by 1
        /// @param idx The index of the voxel within the vector
        /// @return 0 if the voxel was incremented successfully; 1 if the index could not be accesses; or 2 
        ///         if the voxel was already at the maximum count.
        int inc_voxel(const Vector3ui& idx);


        /// @brief Transforms a grid coordinate to the index within the actual vector.
        /// @param grid_idx The X, Y, Z coordinate in the voxel grid.
        /// @return The 1D vector index for the provided 3D grid index.
        /// @note This DOES NOT check validity of that index position within the grid.
        size_t inline grid_idx_to_vector_idx(const Vector3ui& grid_idx);


    public:
        /// If true will round all coordinates in space to the closest voxel, even if they
        /// would have been outside of the voxel space.
        bool round_points_in;


        /// @brief Constructs a voxel representation for the volume between the lower and upper coordinate bounds
        ///        with the given space in each direction.
        /// @param space The number of spaces in the X, Y, and Z directions. Each must be greater than 0.
        /// @param lower The point with the minimum X, Y, and Z coordinates for the voxelized space.
        /// @param upper The point with the maximum X, Y, and Z coordinates for the voxelized space.
        /// @param init Value to initialize all elements to
        /// @param round_points_in Controls how the grid deals with points outside the voxelized space. If true all points
        ///                        will be rounded into the closest voxel.
        /// @throws `invalid_argument` if the spacing in any direction is 0. Or if any upper bound is less than
        ///         the respective lower bound.
        VoxelGrid(Vector3ui space, Eigen::Vector3d lower, Eigen::Vector3d upper, const uint8_t& init = 0, bool round_points_in = true);

        
        /// @brief Sets the given point in space to the provided value
        /// @param point The X, Y, Z coordinate in space.
        /// @param val The value to set the voxel that point is in to. Default is 1.
        /// @return 0 if the point's value was added successfully; or 1 if the point fell outside the voxel grid.
        int add_point(const Eigen::Vector3d& point, const uint8_t& val = 1);

        
        /// @brief Increases the value at the point's location by 1
        /// @param point The X, Y, Z coordinate in space.
        /// @return 0 if the point's value was added successfully; or 1 if the point fell outside the voxel grid.
        int inc_point(const Eigen::Vector3d& point);


        /// @brief Adds equally-spaced points along the line, including the start and end
        /// @param start Coordinate to start from
        /// @param end Coordinate to end at
        /// @param num The number of points to add. Minimum of two
        /// @param surface Value to set elements on the surface (end of the line) to
        /// @param line Value to set elements along the line to.
        /// @return 0 if the line was added without issue
        int add_linear_space(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const size_t& num = 2, const uint8_t &surface = 1, const uint8_t &line = 2);


        /// @brief Adds a line between the start and end point that places approximately one point in each voxel the line touches
        /// @param start Coordinate to start from
        /// @param end Coordinate to end at
        /// @param vox_res TODO: unsure how to treat this tuning variable yet
        /// @param surface Value to set elements on the surface (end of the line) to
        /// @param line Value to set elements along the line to.
        /// @return 0 if the line was added without issue
        /// @note TODO: This is NOT IMPLEMENTED YET
        int add_line_fast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double& vox_res, const uint8_t& surface = 1, const uint8_t& line = 2);


        /// @brief Adds the provided sensor to the grid
        /// @param scanner SimSensorReading object to add
        /// @return 0 if the scanner was added without issue
        int add_sensor(const SimSensorReading& scanner, const uint8_t& surface = 1, const uint8_t& line = 2);


        /// @brief Saves the grids points in a CSV-line format.
        ///        The values are stored incrementing in fastest in  X, then Y, then Z.
        /// @param fname The name for the save file. 
        void save_csv(const std::string& fname);


        /// @brief Saves in the HDF5 Format
        /// @param fname File name
        void save_hdf5(const std::string& fname);


        /// @brief Loads data from an HDF5 file
        /// @param fname File name
        void load_hdf5(const std::string& fname);


        /// @brief Turns coordinates in space into coordinates inside the voxel grid
        /// @param space_xyz The X, Y, Z coordinates in space
        /// @param grid_idx The X, Y, Z returned coordinates in the grid
        /// @return 0 if the grid index is valid, or 1 if it is invalid.
        /// @warning Does not check if the index was out of bounds. See `voxel_in_grid`
        int space_to_grid_idx(const Eigen::Vector3d& space_xyz, Vector3ui& grid_idx);
        

        /// @brief Queries the voxel grid to return the value stored at in the specified space
        /// @param space_xyz The coordinate in space
        /// @return Value at the voxel grid for the point in space. Or 0 if the space was not inside the voxel grid.
        uint8_t space_at(const Eigen::Vector3d& space_xyz);


        /// @brief Queries the space falls within the voxel grid
        /// @param space_xyz The coordinate in space
        /// @return True if the space falls withing the bounds of the voxel grid or False otherwise.
        bool space_in_grid(const Eigen::Vector3d& space_xyz);


        /// @brief Queries the voxel grid to return the value stored at in the specified index
        /// @param voxel_idx The XYZ indices in the voxel grid.
        /// @return Value at the voxel grid for the index. Or 0 if the index was not inside the voxel grid.
        /// @warning Does not check if the index was out of bounds. See `voxel_in_grid`
        uint8_t voxel_at(const Vector3ui& voxel_idx);


        /// @brief Queries the index falls within the voxel grid
        /// @param voxel_idx The XYZ indices in the voxel grid.
        /// @return True if the space falls withing the bounds of the voxel grid or False otherwise.
        bool voxel_in_grid(const Vector3ui& voxel_idx);
};


# endif // MY_VOXEL_H
