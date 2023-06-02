# ifndef FORGESCAN_VOXEL_GRID_PROPERTIES_H
# define FORGESCAN_VOXEL_GRID_PROPERTIES_H

#include <stdexcept>

#include <ForgeScan/forgescan_types.h>


namespace ForgeScan {

/// @brief Storage for VoxelGrid properties.
struct VoxelGridProperties
{
    /// @brief Minimum and maximum truncation distances for rays added to the grid.
    double min_dist = -0.05, max_dist = 0.05;

    /// @brief Voxels per would units in each dimension.
    double resolution = 0.02;

    /// @brief Number of voxels in the X, Y, and Z dimension.
    Vector3ui grid_size = Vector3ui(101, 101, 101);

    /// @brief Size of the grid in world units in the X, Y and Z dimensions; this forms the upper bound.
    Vector3d dimensions = Vector3d(2, 2, 2);

    /// @brief Constructor based on resolution and grid size, dimensions is set implicitly
    /// @param resolution Edge length of each voxel.
    /// @param grid_size  Number of voxels in the grid in each direction. Default (101, 101, 101)
    /// @param min_dist Truncation for minimum distance from a sensed point. Default -0.05.
    /// @param max_dist Truncation for maximum distance from a sensed point. Default +0.05
    /// @throws `std::invalid_argument` If the parameters do not make a valid collection of VoxelGridProperties. See `isValid` for details.
    VoxelGridProperties(const double& resolution = 0.02, const Vector3ui& grid_size = Vector3ui(101, 101, 101), 
                        const double& min_dist = - 0.05, const double& max_dist = 0.05) :
        min_dist(-1 * std::abs(min_dist)),
        max_dist(std::abs(max_dist)),
        resolution(std::abs(resolution)),
        grid_size(grid_size)
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
    /// @param min_dist Truncation for minimum distance from a sensed point. Default -0.05.
    /// @param max_dist Truncation for maximum distance from a sensed point. Default +0.05
    /// @note This constructor may change the final values of `grid_size` and `dimensions` to a be valid pairing with the deduced resolution.
    /// @throws `std::invalid_argument` If the parameters do not make a valid collection of VoxelGridProperties. See `isValid` for details.
    VoxelGridProperties(const Vector3ui& grid_size, const Vector3d& dimensions = Vector3d(2, 2, 2), 
                        const double& min_dist = 0.05, const double& max_dist = 0.05) :
        min_dist(-1 * std::abs(min_dist)),
        max_dist(std::abs(max_dist)),
        grid_size(grid_size),
        dimensions(dimensions)
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

    VoxelGridProperties(const VoxelGridProperties& other) :
        min_dist(other.min_dist), max_dist(other.max_dist), resolution(other.resolution),
        grid_size(other.grid_size), dimensions(other.dimensions)
    {
        isValid();
    }

    VoxelGridProperties& operator=(const VoxelGridProperties& other)
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
            throw std::invalid_argument("[VoxelGridProperties::isValid] Cannot have non-positive resolution.");
        }
        if (std::isnan(max_dist) || std::isnan(min_dist) || max_dist < 0 || min_dist > 0) {
            throw std::invalid_argument("[VoxelGridProperties::isValid] Minimum and maximum distance must both be positive numbers.");
        }
        if ( (dimensions.array() <= 0).any() || dimensions.hasNaN() ) {
            throw std::invalid_argument("[VoxelGridProperties::isValid] Each dimension must be greater than zero.");
        }
        if ( (grid_size.array()  <= 0).any() ) {
            throw std::invalid_argument("[VoxelGridProperties::isValid] The grid size in each direction must be greater than zero.");
        }
        Vector3d dimensions_check = ((grid_size.cast<double>().array() - 1) * resolution);
        if ( !dimensions.isApprox(dimensions_check) ) {
            throw std::invalid_argument("[VoxelGridProperties::isValid] The grid size and resolution do not match the dimensions. " 
                                        "Dimensions should measure from the center of the first voxel (at the origin) to the center of "
                                        "the final voxel (at the dimensions). Is your dimension off by one by one voxel unit?"
                                        "Try calling the setDimensions method.");
        }
        return true;
    }
};

} // ForgeScan

#endif // FORGESCAN_VOXEL_GRID_PROPERTIES_H
