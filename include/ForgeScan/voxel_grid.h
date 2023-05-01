# ifndef FORGESCAN_VOXEL_GRID_H
# define FORGESCAN_VOXEL_GRID_H

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/voxel_element.h>
#include <ForgeScan/sensor_reading.h>
#include <ForgeScan/grid_processor.h>

#include <vector>
#include <stdexcept>


/// @brief Storage for VoxelGrid properties.
/// @note When passed to a VoxelGrid constructor some spatial properties will be adjusted to ensure
///       that, in each direction, the dimension equals the product resolution and the grid size.
/// @note Not all spatial properties (`resolution`, `grid_size`, and `dimensions`) need to be set.
///       The VoxelGrid constructor will use defaults or infer a best value. However, either the 
///       resolution OR the dimensions must be set.
struct VoxelGridProperties
{
    /// @brief Minimum and maximum truncation distances for rays added to the grid.
    double min_dist = 0.05, max_dist = 0.05;

    /// @brief Voxels per would units in each dimension.
    double resolution = 0.01;

    /// @brief Number of voxels in the X, Y, and Z dimension.
    Vector3ui grid_size = Vector3ui(0, 0, 0);

    /// @brief Size of the grid in world units in the X, Y and Z dimensions; this forms the upper bound.
    Vector3d dimensions = Vector3d(1, 1, 1);

    /// @brief Sets minimum and maximum distances to the same value.
    /// @param dist Truncation for minimum and maximum distance from a sensed point.
    VoxelGridProperties(const double& dist) :
        min_dist(-1 * std::abs(dist)),
        max_dist(std::abs(dist))
        { }

    /// @brief Sets minimum and maximum distances to different values.
    /// @param min_dist Truncation for minimum distance from a sensed point.
    /// @param max_dist Truncation for maximum distance from a sensed point.
    VoxelGridProperties(const double& min_dist, const double& max_dist) :
        min_dist(-1 * std::abs(min_dist)),
        max_dist(std::abs(max_dist))
        { }

    bool isValid() {
        return resolution >= 0 || (dimensions.array() > 0).all();
    }
};


/// @brief Container for a 3 dimensional grid of VoxelElements with its own rigid body reference frame.
/// @note  The transformation is between the world and the grid's lower bound, not the center.
class VoxelGrid : public ForgeScanEntity
{
public:
    /// @brief Properties for the voxel grid: spatial properties and truncation distance.
    const VoxelGridProperties properties;

public:
    /// @brief Constructs grid 
    /// @param properties 
    VoxelGrid(const VoxelGridProperties& properties);

    /// @brief 
    /// @param extr 
    /// @param properties 
    VoxelGrid(const VoxelGridProperties& properties, const extrinsic& extr);

    /// @brief 
    /// @param position 
    /// @param properties 
    VoxelGrid(const VoxelGridProperties& properties, const translation& position);

    /// @brief 
    /// @param orientation 
    /// @param properties 
    VoxelGrid(const VoxelGridProperties& properties, const rotation& orientation);

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

    /// @brief Adds the measurements from the sensor to the VoxelGrid, performing required coordinate transformations.
    /// @tparam T Templated on different DepthSensor types.
    /// @param sensor Sensor with measurements to add.
    template <typename T>
    void addSensor(const BaseDepthSensor<T> &sensor)
    {
        /// Get the sensor's measured points, relative to the sensor frame, then transform 
        /// these points from the sensor frame to this VoxelGrid's frame.
        point_list points = sensor.getAllPositions();
        fromOtherToThis(sensor, points);

        /// Get the sensor's position (for the start of each ray) relative to the world frame, then
        /// transform this to the VoxelGrid's frame.
        point sensor_pose = sensor.extr.translation();
        fromWorldToThis(sensor_pose);

        /// The points variables is a 3xN matrix, add each one.
        for (int i = 0, n = points.cols(); i < n; ++i) {
            addRayTSDFandView(*this, sensor_pose, points.col(i));
        }

        /// Reset any element viewed flags.
        for (auto& element : voxel_element_vector)
        {
            if (element.views >> 15)
            {   /// Checks if the MSB of the views is set to 1 and 
                if (element.views != 0xFFFF) ++element.views;   /// Caps updates to prevent rollover after 0x7FFF (32767 views.)
                resetViewUpdateFlag(element);
            }
        }
    }


    /// @brief Rests all data in the grid to zero or its respective defaults.
    void clear();

    /// @brief Saves the grids points in a CSV format.
    ///        The values are stored incrementing in fastest in  X, then Y, then Z and each line is a comma
    ///        separate list of attributes. the first line is a header with what each attribute is
    /// @param fname The name for the save file.  Automatically adds ".csv" when writing.
    /// @details This format is available for ease of data introspection. It simply writes the voxel data
    ///          out but provides little detail about the VoxelGrid's parameters.
    /// @note    No read method is available for this format.
    void saveCSV(const std::string& fname) const;


    /// @brief Saves in the XDMF format (XDMF file references to an HDF5 data file). 
    /// @param fname File name. Automatically adds ".h5" when writing the HDF5 file and ".xdmf"
    ///              to the XDMF file of the same name.
    /// @details This format makes it easier to visualize in tools like ParaView with the built-in
    ///          XDMF readers. But is slightly less efficient to save.
    /// @note    No read method is available for this format.
    void saveXDMF(const std::string& fname) const;


    /// @brief Saves in the HDF5 format.
    /// @param fname File name. Automatically adds ".h5" when writing.
    /// @details This is the fastest save method and the recommended one if the grid is to be re-loaded
    ///          into a VoxelGrid object.
    void saveHDF5(const std::string& fname) const;


    /// @brief Loads data from an HDF5 file.
    /// @param fname File name. Automatically adds ".h5" when searching.
    void loadHDF5(const std::string& fname);

public:
    /// @brief Accesses voxel elements operations on the voxels.
    friend class GridProcessor;

    /// @brief Accesses voxel elements for ray tracing.
    friend void addRayTSDFandView(VoxelGrid &, const point &, const point &);

    /// @brief Accesses voxel elements for ray tracing.
    friend bool addRayExact(VoxelGrid&, const VoxelUpdate&, const point&, const point&, const double&, const double&);


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

    /// @brief Helper function to write the XDMF file.
    /// @param fname Name for the XDMF file.
    void writeXDMF(const std::string &fname) const;

    /// @brief Helper that all constructors call. Populates the vector and checks memory usage,
    ///        printing to console if more than 100 MB are used.
    void setup();
};

#endif // FORGESCAN_VOXEL_GRID_H
