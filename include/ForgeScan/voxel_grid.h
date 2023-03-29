# ifndef FORGESCAN_VOXEL_GRID_H
# define FORGESCAN_VOXEL_GRID_H

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/voxel_element.h>
#include <ForgeScan/sim_sensor_reading.h>
#include <ForgeScan/grid_processor.h>

#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <memory>

#define INVALID_INDEX_ERROR_CODE -1


/// @brief Container for a 3 dimensional grid ov voxels.
class VoxelGrid
{
    friend class GridProcessor;

    friend bool addRayExact(VoxelGrid&, const VoxelUpdate&, const point&, const point&,
                            const double&, const double&);

    friend bool addRayLinspace(VoxelGrid&, const VoxelUpdate&, const point&, const point&, const size_t&);

    friend bool addRayApprox(VoxelGrid&, const VoxelUpdate&, const point&, const point&, const double&);

    private:
        /// Vector structure for the voxels. Increments fastest in X, then Y, then Z.
        std::shared_ptr<std::vector<VoxelElement>> grid;

        /// Size the the grid in each direction; the number of voxels of the specified resolution needed to
        /// span the distance starting from the lower bound to include the upper bound. 
        Vector3ui size;

        /// Edge length of each voxel cube
        double resolution;

        /// Lower (X, Y, Z) bound for coordinates inside of grid
        point lower;

        /// Upper (X, Y, Z) bound for coordinates inside of grid
        point upper;

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
        /// @param round_points_in Controls how the grid deals with points outside the voxelized space. If true all points
        ///                        will be rounded into the closest voxel. Default is true.
        /// @throws `invalid_argument` if the difference between upper and lower in any direction less than or equal to 0.
        /// @throws `invalid argument` if resolution is less than or equal to 0.
        ///         the respective lower bound.
        /// @note The "true" upper bound may be greater than what is given based on the resolution.
        /// @TODO: The default action to round points in will be changed to false in a future update.
        VoxelGrid(double resolution, point lower, point upper, bool round_points_in = true);


        /// @brief Transforms the input index into coordinates within the voxel grid.
        /// @param input Index location within the underlying std::vector
        /// @param output Index in discrete voxel grid
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        /// @warning Transforming something from a vector index is hazardous. For space and grid
        ///          indicies that are out of bounds in X or Y directions, the calculated vector index
        ///          will be a valid alias for a point which is within the grid but with a different Z index. 
        /// @note Because of the above warning this function may be removed in the future.
        int toGrid(const vector_idx& input, grid_idx& output) const;


        /// @brief Transforms the input index into coordinates within the voxel grid.
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param output Index in discrete voxel grid
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int toGrid(const point& input, grid_idx& output) const;


        /// @brief Transforms the input index into coordinates within the continuous space that the grid spans
        /// @param input Index location within the underlying std::vector
        /// @param output Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        /// @warning Transforming something from a vector index is hazardous. For space and grid
        ///          indicies that are out of bounds in X or Y directions, the calculated vector index
        ///          will be a valid alias for a point which is within the grid but with a different Z index. 
        /// @note Because of the above warning this function may be removed in the future.
        int toPoint(const vector_idx& input, point& output) const;


        /// @brief Transforms the input index into coordinates within the continuous space that the grid spans
        /// @param input Index in discrete voxel grid
        /// @param output Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int toPoint(const grid_idx& input, point& output) const;


        /// @brief Transforms the input index into the index location within the underlying std::vector
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param output Index location within the underlying std::vector
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int toVector(const point& input, vector_idx& output) const;


        /// @brief Transforms the input index into the index location within the underlying std::vector
        /// @param input Index in discrete voxel grid
        /// @param output Index location within the underlying std::vector
        /// @returns 0 if the transformation is valid.
        /// @returns A non-zero integer if the transformation is invalid.
        int toVector(const grid_idx& input, vector_idx& output) const;


        /// @brief Checks that the given coordinates are within the grids coordinate bounds
        /// @param input Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @return True if all values are within their respective upper and lower bounds, false otherwise.
        bool inline valid(const point input) const {
            return ( (this->lower.array() <= input.array()).all() && (input.array() <= this->upper.array()).all() );
        }


        /// @brief Checks that the given grid indicies are within the grids index bounds 
        /// @param input Index in discrete voxel grid
        /// @return True if all values are within their respective upper and lower bounds, false otherwise.
        bool inline valid(const grid_idx input) const {
            return (input.array() < this->size.array()).all();
        }


        /// @brief Checks that the given vector index is within the vector bounds
        /// @param input Index location within the underlying std::vector
        /// @return True if the value is less than the vector's size, false otherwise.
        bool inline valid(const vector_idx& input) const {
            return input < this->grid->size();
        }


        /// @brief Returns the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @return Value at the location
        /// @throws `std::invalid_argument` if the vector index is out of bounds. 
        VoxelElement inline at(const point& idx) const {
            vector_idx vidx;
            this->toVector(idx, vidx);
            return this->at(vidx);       
        }


        /// @brief Returns the value at the given location
        /// @param idx Index in discrete voxel grid
        /// @return Value at the location
        /// @throws `std::invalid_argument` if the vector index is out of bounds. 
        VoxelElement inline at(const grid_idx& idx) const {
            vector_idx vidx;
            this->toVector(idx, vidx);
            return this->at(vidx);
        }


        /// @brief Returns the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @return Value at the index
        /// @throws `std::invalid_argument` if the vector index is out of bounds. 
        VoxelElement inline at(const vector_idx& idx) const {
            if (!this->valid(idx)) throw std::invalid_argument("Input resulted in out of bound vector access.");
            return this->grid->at(idx);
        }


        /// @brief Sets the value at the given location
        /// @param idx Coordinates in continuous space relative to the VoxelGrid's reference frame
        /// @param val Value to set to
        /// @returns 0 if the set was successful.
        /// @returns A non-zero integer if the location is invalid.
        int inline set(const point& idx, const VoxelUpdate& val) {
            vector_idx vidx;
            int result = this->toVector(idx, vidx);
            if (result != 0) return INVALID_INDEX_ERROR_CODE;
            return this->set(vidx, val);
        }


        /// @brief Sets the value at the given location
        /// @param idx Index in discrete voxel grid
        /// @param val Value to set to
        /// @returns 0 if the set was successful.
        /// @returns A non-zero integer if the location is invalid.
        int inline set(const grid_idx& idx, const VoxelUpdate& val) {
            vector_idx vidx;
            int result = this->toVector(idx, vidx);
            if (result != 0) return INVALID_INDEX_ERROR_CODE;
            return this->set(vidx, val);
        }


        /// @brief Sets the value at the given location
        /// @param idx Index location within the underlying std::vector
        /// @param val Value to set to
        /// @returns 0 if the set was successful.
        /// @returns A non-zero integer if the transformation is invalid.
        int inline set(const vector_idx& idx, const VoxelUpdate& val) {
            if ( !this->valid(idx) ) return INVALID_INDEX_ERROR_CODE;
            updateVoxel(this->grid->at(idx), val);
            return 0;
        }


        /// @brief Gets the six connected neighbors.
        /// @param input  Index in discrete voxel grid
        /// @param output Neighboring indicies.
        /// @returns 0 if all neighbors are believed to be valid.
        /// @returns A non-zero integer if at least one neighbor is believed to be invalid invalid.
        /// @note This does not guarantee that the output neighbors are all valid elements. But does guarantee 6
        ///       grid indicies will be returned. Call `valid` on the elements to verify that they may be accessed.
        int get_6(const grid_idx& input, std::vector<grid_idx>& output);


        /// @brief Adds the provided sensor to the grid
        /// @param scanner SimSensorReading object to add
        /// @return 0 if the scanner was added without issue
        int add_sensor(const SimSensorReading& scanner, const uint8_t& surface = 1, const uint8_t& line = 2);


        /// @brief Saves the grids points in a CSV-line format.
        ///        The values are stored incrementing in fastest in  X, then Y, then Z.
        /// @param fname The name for the save file. 
        void save_csv(const std::string& fname) const;


        /// @brief Saves in the HDF5 Format
        /// @param fname File name
        void save_hdf5(const std::string& fname) const;


        /// @brief Loads data from an HDF5 file
        /// @param fname File name
        void load_hdf5(const std::string& fname);
};


# endif // FORGESCAN_VOXEL_GRID_H
