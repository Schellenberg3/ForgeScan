# ifndef FORGESCAN_TSDF_GRID_H
# define FORGESCAN_TSDF_GRID_H

#include <filesystem>
#include <vector>
#include <stdexcept>
#include <iostream>

#include "ForgeScan/forgescan_types.h"
#include "ForgeScan/TSDF/voxel.h"
#include "ForgeScan/TSDF/processor.h"
#include "ForgeScan/Metrics/sensor_record.h"
#include "ForgeScan/DepthSensor/sensor.h"
#include "ForgeScan/Utilities/vector_memory_use.h"

namespace ForgeScan {
namespace TSDF      {


/// @brief Container for a 3 dimensional grid of Voxels with its own rigid body reference frame.
/// @note  The transformation is between the world and the grid's lower bound, not the center.
class Grid : public ForgeScanEntity
{
public:

    /// @brief Storage for Grid properties.
    struct Properties
    {
        /// @brief Minimum and maximum truncation distances for rays added to the grid.
        double min_dist = -0.20, max_dist = 0.20;

        /// @brief Voxels per would units in each dimension.
        double resolution = 0.02;

        /// @brief Number of voxels in the X, Y, and Z dimension.
        Vector3ui grid_size = Vector3ui(101, 101, 101);

        /// @brief Size of the grid in world units in the X, Y and Z dimensions; this forms the upper bound.
        Vector3d dimensions = Vector3d(2, 2, 2);

        /// @brief Constructor based on resolution and grid size, dimensions is set implicitly
        /// @param resolution Edge length of each voxel.
        /// @param grid_size  Number of voxels in the grid in each direction. Default (101, 101, 101)
        /// @param min_dist Truncation for minimum distance from a sensed point. Default -0.20.
        /// @param max_dist Truncation for maximum distance from a sensed point. Default +0.20.
        /// @throws `std::invalid_argument` If the parameters do not make a valid collection of Grid Properties. See `isValid` for details.
        Properties(const double& resolution = 0.02, const Vector3ui& grid_size = Vector3ui(101, 101, 101),
                   const double& min_dist = - 0.20, const double& max_dist = 0.20) :
            min_dist(-1 * std::abs(min_dist)), max_dist(std::abs(max_dist)), resolution(std::abs(resolution)), grid_size(grid_size)
            {
                setDimensions();
                isValid();
            }

        /// @brief Constructor based on grid_size and dimensions. Infers resolution based on the smallest resolution required to span a direction.
        ///        This new resolution is used to update the number of voxels required in the other directions and update the dimensions based on
        ///        the number of voxels.
        /// @param grid_size  Number of voxels in the grid.
        /// @param dimensions Size of the grid spanning from the center of the voxel, at the origin, to the center of the voxel at this upper bound.
        ///                   Default is (2, 2, 2).
        /// @param min_dist Truncation for minimum distance from a sensed point. Default -0.20.
        /// @param max_dist Truncation for maximum distance from a sensed point. Default +0.20.
        /// @note This constructor may change the final values of `grid_size` and `dimensions` to a be valid pairing with the deduced resolution.
        /// @throws `std::invalid_argument` If the parameters do not make a valid collection of Grid Properties. See `isValid` for details.
        Properties(const Vector3ui& grid_size, const Vector3d& dimensions = Vector3d(2, 2, 2),
                   const double& min_dist = 0.05, const double& max_dist = 0.05) :
            min_dist(-1 * std::abs(min_dist)), max_dist(std::abs(max_dist)), grid_size(grid_size), dimensions(dimensions)
            {
                /// As we don't know if the dimensions and grid_size align, find the length for the cuboid elements that would span
                /// the dimensions as requested. Take the minimum edge length of this cuboid for our cube voxel.
                this->resolution = (this->dimensions.array() / (this->grid_size.cast<double>().array() - 1) ).minCoeff();

                /// Since the voxel cube will be smaller than the cuboid in at most two directions, we recalculate how many voxels are
                /// needed to span the dimensions, rounding up to be inclusive. We then add one voxel in each direction for proper centering
                /// between the voxels and the dimensions.
                this->grid_size = (dimensions.array() / resolution).ceil().cast<size_t>();
                this->grid_size.array() += 1;

                /// Since we rounded up when recalculating the grid size the dimensions may be off, so we update this now.
                /// Typically this should only increase the dimension by
                setDimensions();
                isValid();
            }

        Properties(const Properties& other) :
            min_dist(other.min_dist), max_dist(other.max_dist), resolution(other.resolution),
            grid_size(other.grid_size), dimensions(other.dimensions)
            { isValid(); }

        Properties& operator=(const Properties& other)
        {
            this->min_dist = other.min_dist;
            this->max_dist = other.max_dist;
            this->resolution = other.resolution;
            this->grid_size = other.grid_size;
            this->dimensions = other.dimensions;
            isValid();
            return *this;
        }

        /// @brief Update the dimensions to fit the grid_size and resolution.
        void setDimensions() { this->dimensions = (grid_size.cast<double>().array() - 1) * this->resolution;}

        /// @brief  Verifies that several conditions are met for the parameters to be valid:
        ///             - `resolution` is greater than zero.
        ///             - `max_dist` is greater than or equal to zero and `min_dist` is less than or equal to zero.
        ///             - `dimensions` is greater than zero in each direction.
        ///             - `grid_size` is greater than zero in each direction.
        ///             - `dimension` equals `(grid_size - 1) * resolution` in each direction.
        /// @return True if all conditions are met.
        /// @throws `std::invalid_argument` If any condition is not met.
        bool isValid() {
            if (resolution <= 0 || std::isnan(resolution)) {
                throw std::invalid_argument("[Grid::Properties::isValid] Cannot have non-positive resolution.");
            }
            if (std::isnan(max_dist) || std::isnan(min_dist) || max_dist < 0 || min_dist > 0) {
                throw std::invalid_argument("[Grid::Properties::isValid] Minimum and maximum distance must both be positive numbers.");
            }
            if ( (dimensions.array() <= 0).any() || dimensions.hasNaN() ) {
                throw std::invalid_argument("[Grid::Properties::isValid] Each dimension must be greater than zero.");
            }
            if ( (grid_size.array()  <= 0).any() ) {
                throw std::invalid_argument("[Grid::Properties::isValid] The grid size in each direction must be greater than zero.");
            }
            Vector3d dimensions_check = ((grid_size.cast<double>().array() - 1) * resolution);
            if ( !dimensions.isApprox(dimensions_check) ) {
                throw std::invalid_argument("[Grid::Properties::isValid] The grid size and resolution do not match the dimensions. "
                                            "Dimensions should measure from the center of the first voxel (at the origin) to the center of "
                                            "the final voxel (at the dimensions). Is your dimension off by one by one voxel unit?"
                                            "Try calling the setDimensions method.");
            }
            return true;
        }
    };

    /// @brief Properties for the voxel grid: spatial properties and truncation distance.
    const Properties properties;

    Metrics::SensorRecord views;

public:
    Grid(const Properties& properties) :
        ForgeScanEntity(), properties(properties),
        p2i_scale((properties.grid_size.cast<double>().array() - 1) / properties.dimensions.array())
        { setup(); }

    Grid(const double& resolution, const Vector3ui& grid_size = Vector3ui(101, 101, 101),
              const double& min_dist = 0.05, const double& max_dist = 0.05) :
        ForgeScanEntity(), properties(resolution, grid_size, min_dist, max_dist),
        p2i_scale((properties.grid_size.cast<double>().array() - 1) / properties.dimensions.array())
        { setup(); }

    Grid(const Vector3ui& grid_size = Vector3ui(101, 101, 101), const Vector3d& dimensions = Vector3d(1, 1, 1),
              const double& min_dist = -0.05, const double& max_dist = 0.05) :
        ForgeScanEntity(),  properties(grid_size, dimensions, min_dist, max_dist),
        p2i_scale((properties.grid_size.cast<double>().array() - 1) / properties.dimensions.array())
        { setup(); }

    /// @brief Checks that the voxel index is valid for the shape of the voxel grid.
    /// @param voxel Index for the desired voxel.
    /// @return True if valid. False otherwise.
    bool valid(const index& voxel) const { return (voxel.array() < properties.grid_size.array()).all(); }

    /// @brief Accesses the voxels with no bounds checking.
    /// @param voxel Index for the desired voxel.
    /// @return Writable access to the specified voxel.
    /// @note Without checking out of bounds lookups may result in undefined behaviour. Use the `at` method
    ///       to access voxels with bounds checking.
    Voxel& operator[](const index& voxel) { return voxel_vector[indexToVector(voxel)]; }

    /// @brief Accesses the voxels with no bounds checking.
    /// @param voxel Index for the desired voxel.
    /// @return Read-only access to the specified voxel.
    /// @note Without checking out of bounds lookups may result in undefined behaviour. Use the `at` method
    ///       to access voxels with bounds checking.
    const Voxel& operator[](const index& voxel) const { return voxel_vector[indexToVector(voxel)]; }

    /// @brief Accesses the voxels with bounds checking.
    /// @param voxel Index for the desired voxel.
    /// @return Writable access to the specified voxel.
    /// @throw `std::out_of_range` if the requested index exceeds the Grid's size in any dimension.
    Voxel& at(const index& voxel) { return voxel_vector.at(indexToVectorThrowOutOfRange(voxel)); }

    /// @brief Accesses the voxels with bounds checking.
    /// @param voxel Index for the desired voxel.
    /// @return Read-only access to the specified voxel.
    /// @throw `std::out_of_range` if the requested index exceeds the Grid's size in any dimension.
    const Voxel& at(const index& voxel) const { return voxel_vector.at(indexToVectorThrowOutOfRange(voxel)); }

    /// @brief Calculates to index that the point falls into within the grid.
    /// @param input Cartesian position of the point, relative to the voxel grid origin.
    /// @return Grid index that the point would be in.
    /// @note The input MUST be transformed to the Grid's coordinate system for valid results.
    /// @note This does not promise that the index is valid. Use `valid` of the returned input to verify the results.
    index pointToIndex(const point& input) const { return (input.array() * p2i_scale).round().cast<size_t>(); }

    /// @brief Calculates to index that the point falls into within the grid.
    /// @param input Cartesian position of the point.
    /// @param extr  Frame which the point is in.
    /// @return Grid index that the point would be in.
    /// @note This does not promise that the index is valid. Use `valid` of the returned input to verify the results.
    index pointToIndex(const point& input, const extrinsic& extr) const
        { return pointToIndex( toThisFromOther(input, extr) ); }

    /// @brief Calculates to index that the point falls into within the grid.
    /// @param input Cartesian position of the point.
    /// @param other Other ForgeScan entity in whose frame the point is expressed.
    /// @param other Frame which the point is in.
    /// @return Grid index that the point would be in.
    /// @note This does not promise that the index is valid. Use `valid` of the returned input to verify the results.
    index pointToIndex(const point& input, const ForgeScanEntity& other) const
        { return pointToIndex( toThisFromOther(input, other.extr) ); }

    /// @brief Calculates the center point location for the voxel at the input index.
    /// @param input The (X, Y, Z) index in the grid to check.
    /// @return Center point of the voxel, relative to the Grid.
    point indexToPoint(const index& input) const { return input.cast<double>().array() * properties.resolution; }

    /// @brief Calculates the center point location for the voxel at the input index.
    /// @param input The (X, Y, Z) index in the grid to check.
    /// @param extr  Frame to transform the resulting point to.
    /// @return Center point of the voxel, relative to the provided frame.
    point indexToPoint(const index& input, const extrinsic& extr) const
        { return toOtherFromThis(indexToPoint(input) , extr); }

    /// @brief Calculates the center point location for the voxel at the input index.
    /// @param input The (X, Y, Z) index in the grid to check.
    /// @param extr  ForgeScanEntity to transform the resulting point to.
    /// @return Center point of the voxel, relative to the provided ForgeScanEntity.
    point indexToPoint(const index& input, const ForgeScanEntity& other) const
        { indexToPoint(input, other.extr); }

    /// @brief Rests all data in the grid to zero or its respective defaults.
    void clear() { for (auto& voxel : voxel_vector) voxel.reset(); }

    /// @brief Updates view count in the voxel grid and resets voxels' viewed flags.
    /// @note  This function is slow! No matter what we must iterate the whole grid. Avoid calling it often.
    void updateViewCount() {
        for (auto& voxel : voxel_vector) {
            if (voxel.views > 0x7FFF && voxel.views != 0xFFFF)
            {   /// Checks if the MSB of the views is set to 1 and prevents rollover after 0x7FFF (32767 views).
                ++voxel.views;
                voxel.resetViewUpdateFlag();
            }
        }
    }

    /// @brief Saves in the XDMF format (XDMF file references to an HDF5 data file).
    /// @param fname File name. Automatically adds ".h5" when writing the HDF5 file and ".xdmf"
    ///              to the XDMF file of the same name.
    /// @details This format makes it easier to visualize in tools like ParaView with the built-in
    ///          XDMF readers. But is slightly less efficient to save.
    /// @note    No read method is available for this format.
    /// @throws `std::invalid_argument` if there is an issue parsing the file name.
    void saveXDMF(const std::filesystem::path& fname) const;

    /// @brief Saves in the HDF5 format.
    /// @param fname File name. Automatically adds ".h5" when writing.
    /// @details This is the fastest save method and the recommended one if the grid is to be re-loaded
    ///          into a Grid object.
    /// @throws `std::invalid_argument` if there is an issue parsing the file name.
    void saveHDF5(const std::filesystem::path& fname) const;

public:
    /// @brief Accesses voxel operations on the voxels.
    friend class Processor;

    friend Grid loadGridHDF5(const std::filesystem::path&);

private:
    /// @brief Container for Voxels. Users see a 3D grid, but this is really just a vector.
    std::vector<Voxel> voxel_vector;

    /// @brief Precomputed voxels-per-dimension-length factor for point to index conversion.
    const Eigen::Array3d p2i_scale;

private:
    /// @brief Retrieves the vector index for the given X, Y, Z index in the grid.
    /// @param voxel Index for the desired voxel.
    /// @return Vector position for the desired voxel.
    /// @note This does not check that the provided pose is valid. Either that the vector position is within
    ///       the bounds of the vector OR that the provided voxel index is valid for the grid size.
    size_t indexToVector(const index voxel) const {
        return voxel[0] + (voxel[1] * properties.grid_size[0]) + (voxel[2] * properties.grid_size[0] * properties.grid_size[1]);
    }

    /// @brief Retrieves the vector index for the given X, Y, Z index in the grid.
    /// @param voxel Index for the desired voxel.
    /// @return Vector position for the desired voxel.
    /// @throw `std::out_of_range` if the requested voxel index exceed the Grid's size in any dimension.
    size_t indexToVectorThrowOutOfRange(const index voxel) const {
        if (!valid(voxel))
            throw std::out_of_range("Requested voxel was not within the bounds of the 3D grid.");
        return indexToVector(voxel);
    }

    /// @brief Helper function to write the XDMF file.
    /// @param fname Name for the XDMF file.
    void writeXDMF(const std::filesystem::path &fname) const;

    /// @brief Helper that all constructors call. Populates the vector and checks memory usage,
    ///        printing to console if more than 100 MB are used.
    void setup() {
        voxel_vector.resize(properties.grid_size.prod());
        double mem = ForgeScan::Utilities::byte_to_megabytes(ForgeScan::Utilities::vector_capacity(voxel_vector));
        if (mem > 100.0)
            std::cout << "Warning, allocated " << mem << " MB for vector grid!" << std::endl;
    }
};

/// @brief  Loads data from an HDF5 file.
/// @param fname File name. Automatically adds ".h5" when searching.
/// @throws `HighFive::Exception` if HighFive encounters an issue while reading the data.
/// @throws `std::invalid_argument` if there is an issue parsing the file name.
Grid loadGridHDF5(const std::filesystem::path& fname);


} // TSDF
} // ForgeScan

#endif // FORGESCAN_TSDF_GRID_H
