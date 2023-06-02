# ifndef FORGESCAN_VOXEL_GRID_H
# define FORGESCAN_VOXEL_GRID_H

#include <filesystem>
#include <vector>
#include <stdexcept>

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/voxel_grid_properties.h>
#include <ForgeScan/voxel_element.h>
#include <ForgeScan/depth_sensor.h>
#include <ForgeScan/grid_processor.h>
#include <ForgeScan/view_tracker.h>


namespace ForgeScan {

/// @brief Container for a 3 dimensional grid of VoxelElements with its own rigid body reference frame.
/// @note  The transformation is between the world and the grid's lower bound, not the center.
class VoxelGrid : public ForgeScanEntity
{
public:
    /// @brief Properties for the voxel grid: spatial properties and truncation distance.
    const VoxelGridProperties properties;

    ViewTracker views;

public:
    VoxelGrid(const VoxelGridProperties& properties) :
        ForgeScanEntity(),
        properties(properties)
    {
        setup();
    }

    VoxelGrid(const double& resolution, const Vector3ui& grid_size = Vector3ui(101, 101, 101),
              const double& min_dist = 0.05, const double& max_dist = 0.05) :
        ForgeScanEntity(),
        properties(resolution, grid_size, min_dist, max_dist)
    {
        setup();
    }

    VoxelGrid(const Vector3ui& grid_size = Vector3ui(101, 101, 101), const Vector3d& dimensions = Vector3d(1, 1, 1),
              const double& min_dist = -0.05, const double& max_dist = 0.05) :
        ForgeScanEntity(),
        properties(grid_size, dimensions, min_dist, max_dist)
    {
        setup();
    }

    /// @brief Checks that the voxel indicies are valid for the shape of the voxel grid.
    /// @param voxel Indicies for the desired voxel.
    /// @return True if valid. False otherwise.
    bool valid(const grid_idx& voxel) const { return (voxel.array() < properties.grid_size.array()).all(); }

    /// @brief Accesses the voxels with no bounds checking.
    /// @param voxel Indicies for the desired voxel.
    /// @return Writable access to the specified voxel element.
    /// @note Without checking out of bounds lookups may result in undefined behaviour. Use the `at` method
    ///       to access elements with bounds checking.
    VoxelElement& operator[](const grid_idx& voxel) { return voxel_element_vector[indiciesToVector(voxel)]; }

    /// @brief Accesses the voxels with no bounds checking.
    /// @param voxel Indicies for the desired voxel.
    /// @return Read-only access to the specified voxel element.
    /// @note Without checking out of bounds lookups may result in undefined behaviour. Use the `at` method
    ///       to access elements with bounds checking.
    const VoxelElement& operator[](const grid_idx& voxel) const { return voxel_element_vector[indiciesToVector(voxel)]; }

    /// @brief Accesses the voxels with bounds checking.
    /// @param voxel Indicies for the desired voxel.
    /// @return Writable access to the specified voxel element.
    /// @throw `std::out_of_range` if the requested voxel indicies exceed the VoxelGrid's size in any dimension. 
    VoxelElement& at(const grid_idx& voxel) { return voxel_element_vector.at(indiciesToVectorThrowOutOfRange(voxel)); }

    /// @brief Accesses the voxels with bounds checking.
    /// @param voxel Indicies for the desired voxel.
    /// @return Read-only access to the specified voxel element.
    /// @throw `std::out_of_range` if the requested voxel indicies exceed the VoxelGrid's size in any dimension. 
    const VoxelElement& at(const grid_idx& voxel) const { return voxel_element_vector.at(indiciesToVectorThrowOutOfRange(voxel)); }

    /// @brief Calculates to grid indicies that the point falls into.
    /// @param input Cartesian position of the point, relative to the voxel grid origin.
    /// @return Grid indicies that the point would be in.
    /// @note The input MUST be transformed to the VoxelGrid's coordinate system for valid results.
    /// @note This does not promise that the index is valid. Use `valid` of the returned input to verify the results.
    grid_idx pointToGrid(const point& input) const {
        Vector3d temp = input.array() * ( properties.grid_size.cast<double>().array() / properties.dimensions.array() );
        temp.array().floor();
        return temp.cast<size_t>();
    }

    /// @brief Updates voxel on the line between the two specified points.
    ///        Points are assumed to be relative to the world frame.
    /// @param update VoxelUpdate to apply to each voxel on the ray.
    /// @param rs Ray start position, world coordinates.
    /// @param re Ray end position, world coordinates.
    /// @returns False if the ray did not intersect the voxel grid. True otherwise.
    bool addRayExact(const VoxelUpdate& update, const point& rs, const point& re) {
        point rs_this = fromWorldToThis(rs);
        point re_this = fromWorldToThis(re);
        return implementAddRayExact(update, rs_this, re_this);
    }


    /// @brief Adds data to the grid, updating voxels near the sensed point with truncated distance and marking the voxels
    ///        between the origin and positive truncation as viewed. Points are assumed to be relative to the world frame.
    /// @param grid The VoxelGrid to add data to.
    /// @param origin Origin for the ray, world coordinates.
    /// @param sensed Sensed point, world coordinates.
    /// @returns False if the ray did not intersect the voxel grid. True otherwise.
    bool addRayTSDF(const point &origin, const point &sensed) {
        point origin_this = fromWorldToThis(origin);
        point sensed_this = fromWorldToThis(sensed);
        RayRecord ray_record; // Needed for implementation, but unused in this function.
        return implementAddRayTSDF(origin_this, sensed_this, ray_record);
    }

    /// @brief Adds the measurements from the sensor to the VoxelGrid, performing required coordinate transformations.
    /// @param sensor Sensor with measurements to add.
    void addSensor(const DepthSensor::BaseDepthSensor&sensor)
    {
        /// Get the sensor's measured points, relative to the sensor frame, then transform 
        /// these points from the sensor frame to this VoxelGrid's frame.
        point_list points = sensor.getAllPositions();
        fromOtherToThis(sensor, points);

        /// Get the sensor's position (for the start of each ray) relative to the world frame, then
        /// transform this to the VoxelGrid's frame.
        point sensor_pose = sensor.extr.translation();
        fromWorldToThis(sensor_pose);

        /// Set up tracking objects for the ViewTracker.
        SensorRecord sensor_record( getTransformationTo(sensor), sensor.intr->size() );
        RayRecord ray_record;

        /// The points variables is a 3xN matrix, add each one.
        auto n = points.cols();
        for (int i = 0; i < n; ++i) {
            ray_record.reset();
            implementAddRayTSDF(sensor_pose, points.col(i), ray_record);
            sensor_record += ray_record;
        }
        views.add(sensor_record);
        updateViewCount();
    }


    /// @brief Rests all data in the grid to zero or its respective defaults.
    void clear();

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
    ///          into a VoxelGrid object.
    /// @throws `std::invalid_argument` if there is an issue parsing the file name.
    void saveHDF5(const std::filesystem::path& fname) const;

public:
    /// @brief Accesses voxel elements operations on the voxels.
    friend class GridProcessor;

friend VoxelGrid loadVoxelGridHDF5(const std::filesystem::path&);

private:
    /// @brief Container for VoxelElements. Users see a 3D grid, but this is really just a vector.
    std::vector<VoxelElement> voxel_element_vector;


private:
    /// @brief Retrieves the vector index for the given X, Y, Z indicies in the voxel grid.
    /// @param voxel Indicies for the desired voxel.
    /// @return Vector position for the desired voxel.
    /// @note This does not check that the provided pose is valid. Either that the vector position is within
    ///       the bounds of the vector OR that the provided voxel index is valid for the grid size.
    size_t indiciesToVector(const grid_idx voxel) const {
        return voxel[0] + (voxel[1] * properties.grid_size[0]) + (voxel[2] * properties.grid_size[0] * properties.grid_size[1]);
    }

    /// @brief Retrieves the vector index for the given X, Y, Z indicies in the voxel grid.
    /// @param voxel Indicies for the desired voxel.
    /// @return Vector position for the desired voxel.
    /// @throw `std::out_of_range` if the requested voxel indicies exceed the VoxelGrid's size in any dimension. 
    size_t indiciesToVectorThrowOutOfRange(const grid_idx voxel) const {
        if (!valid(voxel))
            throw std::out_of_range("Requested voxel was not within the bounds of the 3D grid.");
        return indiciesToVector(voxel);
    }

    /// @brief Updates voxel on the line between the two specified points. Points are in the VoxelGrid's frame.
    /// @details Actual implementation of ray tracing. Defined in `src/voxel_grid_traversal.cpp`.
    /// @param update VoxelUpdate to apply to each voxel on the ray.
    /// @param rs Ray start position, local coordinates.
    /// @param re Ray end position, local coordinates.
    /// @returns False if the ray did not intersect the voxel grid. True otherwise.
    bool implementAddRayExact(const VoxelUpdate& update, const point& rs, const point& re);

    /// @brief Adds data to the grid, updating voxels near the sensed point with truncated distance and marking
    ///        the voxels between the origin and positive truncation as viewed.
    /// @details Actual implementation of ray tracing. Defined in `src/voxel_grid_traversal.cpp`.
    /// @param origin Origin for the ray, local coordinates.
    /// @param sensed Sensed point, local coordinates.
    /// @param ray_record Output variable. Statistics about how this ray changed the VoxelGrid.
    /// @returns False if the ray did not intersect the voxel grid. True otherwise.
    bool implementAddRayTSDF(const point &origin, const point &sensed, RayRecord& ray_record);

    /// @brief Updates view count in the voxel grid and resets element viewed flags.
    void updateViewCount() {
        for (auto& element : voxel_element_vector) {
            if (element.views >> 15) 
            {   /// Checks if the MSB of the views is set to 1. 
                if (element.views != 0xFFFF) ++element.views;   /// Caps updates to prevent rollover after 0x7FFF (32767 views).
                element.resetViewUpdateFlag();
            }
        }
    }

    /// @brief Helper function to write the XDMF file.
    /// @param fname Name for the XDMF file.
    void writeXDMF(const std::filesystem::path &fname) const;

    /// @brief Helper that all constructors call. Populates the vector and checks memory usage,
    ///        printing to console if more than 100 MB are used.
    void setup();
};

/// @brief  Loads data from an HDF5 file.
/// @param fname File name. Automatically adds ".h5" when searching.
/// @throws `HighFive::Exception` if HighFive encounters an issue while reading the data.
/// @throws `std::invalid_argument` if there is an issue parsing the file name.
VoxelGrid loadVoxelGridHDF5(const std::filesystem::path& fname);

} // ForgeScan

#endif // FORGESCAN_VOXEL_GRID_H
