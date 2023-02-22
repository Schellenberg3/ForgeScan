# ifndef FORGESCAN_VOXEL_GRID_H
# define FORGESCAN_VOXEL_GRID_H

#include <ForgeScan/sim_sensor_reading.h>
#include <ForgeScan/grid_processor.h>

#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <memory>

#define INVALID_INDEX_ERROR_CODE -1


/// Convenience typedef for Eigen
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3i Vector3i;

/// Convenience typedef for Eigen; 32 bit unsigned
typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;


/// @brief Container for a 3 dimensional grid ov voxels.
class VoxelGrid
{
    friend class GridProcessor;


    private:
        /// Vector structure for the voxels. Increments fastest in X, then Y, then Z.
        std::shared_ptr<std::vector<uint8_t>> grid;

        /// Size the the grid in each direction; the number of voxels of the specified resolution needed to
        /// span the distance starting from the lower bound to include the upper bound. 
        Vector3ui size;

        /// Edge length of each voxel cube
        double resolution;

        /// Lower (X, Y, Z) bound for coordinates inside of grid
        Eigen::Vector3d lower;

        /// Upper (X, Y, Z) bound for coordinates inside of grid
        Vector3d upper;

        /// Pre-computed scaling factor for point placement
        Vector3d idx_scale;

    public:
        /// If true will round all coordinates to the closest voxel, even if they would have been outside
        /// the bounds of the voxelized space.
        /// @note This functionality may be removed in the future 
        bool round_points_in;


        /// @brief Constructs a discretized voxel representation of the volume between the given bounds at the given resolution
        /// @param resolution The edge length of each voxel. Must be greater than 0.
        /// @param lower The minimum bounds for the discretized volume.
        /// @param upper The maximum bounds for the discretized volume.
        /// @param init Value to initialize all elements to
        /// @param round_points_in Controls how the grid deals with points outside the voxelized space. If true all points
        ///                        will be rounded into the closest voxel. Default is true.
        /// @throws `invalid_argument` if the difference between upper and lower in any direction less than or equal to 0.
        /// @throws `invalid argument` if resolution is less than or equal to 0.
        ///         the respective lower bound.
        /// @note The "true" upper bound may be greater than what is given based on the resolution.
        /// @TODO: The default action to round points in will be changed to false in a future update.
        VoxelGrid(double resolution, Eigen::Vector3d lower, Eigen::Vector3d upper, const uint8_t& init = 0, bool round_points_in = true);


        /// @brief Transforms the input index into coordinates within the voxel grid.
        /// @param input Index location within the underlying std::vector
        /// @param output Index in discrete voxel grid
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        /// @warning Transforming something from a vector index is hazardous. For space and grid
        ///          indicies that are out of bounds in X or Y directions, the calculated vector index
        ///          will be a valid alias for a point which is within the grid but with a different Z index. 
        /// @note Because of the above warning this function may be removed in the future.
        int gidx(const size_t& input, Vector3ui& output);


        /// @brief Transforms the input index into coordinates within the voxel grid.
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param output Index in discrete voxel grid
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int gidx(const Vector3d& input, Vector3ui& output);


        /// @brief Transforms the input index into coordinates within the continuous space that the grid spans
        /// @param input Index location within the underlying std::vector
        /// @param output Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        /// @warning Transforming something from a vector index is hazardous. For space and grid
        ///          indicies that are out of bounds in X or Y directions, the calculated vector index
        ///          will be a valid alias for a point which is within the grid but with a different Z index. 
        /// @note Because of the above warning this function may be removed in the future.
        int sidx(const size_t& input, Vector3d& output);


        /// @brief Transforms the input index into coordinates within the continuous space that the grid spans
        /// @param input Index in discrete voxel grid
        /// @param output Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int sidx(const Vector3ui& input, Vector3d& output);


        /// @brief Transforms the input index into the index location within the underlying std::vector
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param output Index location within the underlying std::vector
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int vidx(const Vector3d& input, size_t& output);


        /// @brief Transforms the input index into the index location within the underlying std::vector
        /// @param input Index in discrete voxel grid
        /// @param output Index location within the underlying std::vector
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int vidx(const Vector3ui& input, size_t& output);


        /// @brief Checks that the given coordinates are within the grids coordinate bounds
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @return True if all values are within their respective upper and lower bounds, false otherwise.
        bool const inline valid(const Vector3d input) {
            return (this->lower.array() <= input.array() && input.array() <= this->upper.array()).all();
        }


        /// @brief Checks that the given grid indicies are within the grids index bounds 
        /// @param input Index in discrete voxel grid
        /// @return True if all values are within their respective upper and lower bounds, false otherwise.
        bool const inline valid(const Vector3ui input) {
            return (input.array() < this->size.array()).all();
        }


        /// @brief Checks that the given vector index is within the vector bounds
        /// @param input Index location within the underlying std::vector
        /// @return True if the value is less than the vector's size, false otherwise.
        bool const inline valid(const size_t& input) {
            return input < this->grid->size();
        }


        /// @brief Returns the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @return Value at the location
        /// @throws `std::invalid_argument` if the vector index is out of bounds. 
        uint8_t const inline at(const Vector3d& idx) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->at(vidx);       
        }


        /// @brief Returns the value at the given location
        /// @param idx Index in discrete voxel grid
        /// @return Value at the location
        /// @throws `std::invalid_argument` if the vector index is out of bounds. 
        uint8_t const inline at(const Vector3ui& idx) {
            // TODO: This and the 
            size_t vidx;
            this->vidx(idx, vidx);
            return this->at(vidx);
        }


        /// @brief Returns the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @return Value at the index
        /// @throws `std::invalid_argument` if the vector index is out of bounds. 
        uint8_t const inline at(const size_t& idx) {
            if (!this->valid(idx)) throw std::invalid_argument("Input resulted in out of bound vector access.");
            return this->grid->at(idx);
        }


        /// @brief Sets the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param val Value to set to
        /// @returns 0 if the set was successful.
        /// @returns A non-zero integer if the location is invalid.
        int inline set(const Vector3d& idx, const uint8_t& val) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->set(vidx, val);
        }


        /// @brief Sets the value at the given location
        /// @param idx Index in discrete voxel grid
        /// @param val Value to set to
        /// @returns 0 if the set was successful.
        /// @returns A non-zero integer if the location is invalid.
        int inline set(const Vector3ui& idx, const uint8_t& val) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->set(vidx, val);
        }


        /// @brief Sets the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @param val Value to set to
        /// @returns 0 if the set was successful.
        /// @returns A non-zero integer if the transformation is invalid.
        int inline set(const size_t& idx, const uint8_t& val) {
            if ( !this->valid(idx) ) return INVALID_INDEX_ERROR_CODE;
            this->grid->at(idx) = val;
            return 0;
        }


        /// @brief Increments the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param val Value to Increments by. May be negative.
        /// @returns 0 if the increment was successful.
        /// @returns A non-zero integer if the location is invalid.
        /// @warning This does not check for overflow or underflow.
        int inline inc(const Vector3d& idx, const uint8_t& val = 1) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->inc(vidx, val);       
        }


        /// @brief Increments the value at the given location
        /// @param idx Index in discrete voxel grid
        /// @param val Value to Increments by. May be negative.
        /// @returns 0 if the increment was successful.
        /// @returns A non-zero integer if the location is invalid.
        /// @warning This does not check for overflow or underflow.
        int inline inc(const Vector3ui& idx, const int& val = 1) {
            size_t vidx;
            this->vidx(idx, vidx);
            return this->inc(vidx, val);       
        }


        /// @brief Increments the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @param val Value to Increments by. May be negative.
        /// @returns 0 if the increment was successful.
        /// @returns A non-zero integer if the location is invalid.
        /// @warning This does not check for overflow or underflow.
        int inline inc(const size_t& idx, const int& val = 1) {
            if ( !this->valid(idx) ) return INVALID_INDEX_ERROR_CODE;
            this->grid->at(idx) += val;
            return 0;
        }


        /// @brief Gets the six connected neighbors.
        /// @param input  Index in discrete voxel grid
        /// @param output Neighboring indicies.
        /// @returns 0 if all neighbors are believed to be valid.
        /// @returns A non-zero integer if at least one neighbor is believed to be invalid invalid.
        /// @note This does not guarantee that the output neighbors are all valid elements. But does guarantee 6
        ///       grid indicies will be returned. Call `valid` on the elements to verify that they may be accessed.
        int get_6(const Vector3ui& input, std::vector<Vector3ui>& output);


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
};


# endif // FORGESCAN_VOXEL_GRID_H
