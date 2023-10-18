#ifndef DEMOS_CPP_FORGE_SCAN_RECONSTRUCTION_VOXEL_GRID_HPP
#define DEMOS_CPP_FORGE_SCAN_RECONSTRUCTION_VOXEL_GRID_HPP

#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>

#include "ForgeScan/Common/Definitions.hpp"
#include "ForgeScan/Common/Exceptions.hpp"
#include "ForgeScan/Common/Grid.hpp"
#include "ForgeScan/Common/VoxelData.hpp"
#include "ForgeScan/Common/Types.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Files.hpp"
#include "ForgeScan/Utilities/MemoryUse.hpp"
#include "ForgeScan/Utilities/XDMF.hpp"


namespace forge_scan {
namespace data {

    // Forward definition to allow friend access.
    class Reconstruction;

} // namespace data
} // namespace forge_scan


namespace forge_scan {
namespace data {


/// @brief A base class for a VoxelGrid of various data types. Derived classes define what datatypes they accept
///        and implement strategies for updating with new data.
class VoxelGrid : public Grid
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @details Required to call the save method.
    friend class Reconstruction;


public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    virtual ~VoxelGrid() { }

    /// @return Help message for constructing a derived VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: VoxelGrid help message";
    }


    /// @brief Provides the VoxelGrid with a view of the `data::Reconstruction`'s data_seen record.
    /// @param data_seen Vector of booleans the same length as `VoxelGrid::data`.
    void addSeenData(const std::shared_ptr<std::vector<bool>>& data_seen)
    {
        this->data_seen = data_seen;
    }


    /// @brief Calculates how much space the data vector is using.
    /// @param [out] size_bytes     Number of bytes used by the vector.
    /// @param [out] capacity_bytes Number of bytes used by the vector, including capacity.
    virtual void getDataMemoryUsage(size_t& size_bytes, size_t& capacity_bytes) const
    {
        auto visit_memory_usage = [&size_bytes, &capacity_bytes](auto&& vec)
        {
            size_bytes     = utilities::memory_use::vector_size(vec);
            capacity_bytes = utilities::memory_use::vector_capacity(vec);
        };
        std::visit(visit_memory_usage, this->data);
    }


    /// @brief Calculates how much space the data vector is using.
    /// @param [out] size_mb     Number of megabytes used by the vector.
    /// @param [out] capacity_mb Number of megabytes used by the vector, including capacity.
    virtual void getDataMemoryUsage(float& size_mb, float& capacity_mb) const
    {
        size_t size_bytes, capacity_bytes;
        this->getDataMemoryUsage(size_bytes, capacity_bytes);
        size_mb     = utilities::memory_use::byte_to_megabytes(size_bytes);
        capacity_mb = utilities::memory_use::byte_to_megabytes(capacity_bytes);
    }


    /// @brief Gets a constant reference to the VectorVariant data the VoxelGrid stores.
    /// @return Read-only reference to the data.
    const VectorVariant& getData() const
    {
        return this->data;
    }

    /// @brief Runs after calls to `update` so a grid can preform post-processing on its data, if needed.
    virtual void postUpdate()
    {

    }


    // ***************************************************************************************** //
    // *                             PUBLIC PURE VIRTUAL METHODS                               * //
    // ***************************************************************************************** //


    /// @brief Updates the VoxelGrid with new information along a ray.
    /// @param ray_trace A trace to update the VoxelGrid along.
    virtual void update(const std::shared_ptr<const Trace>& ray_trace) = 0;



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief The minimum and maximum distance along a trace that the VoxelGrid will perform
    ///        updates at. A minimum of 0 means the Grid will start at the sensed point. A maximum
    ///        of infinity means the Grid will update until the last voxel on the trace (the ray's
    ///        origin or the Grid's boundary). Negative values are valid too.
    ///        The VoxelGrid constructor guarantees that dist_min <= dist_max.
    const float dist_min, dist_max;

    /// @brief Variant type which holds the default value for the VoxelGrid.
    ///        All elements are initialized to this value.
    DataVariant default_value;


    static const float default_zero, default_infinity, default_d_min, default_d_max;

    static const std::string parse_type, parse_d_min, parse_d_max, parse_default, parse_dtype;

    static const std::string type_name;

protected:
    // ***************************************************************************************** //
    // *                               PROTECTED NESTED CLASS                                  * //
    // ***************************************************************************************** //


    /// @brief A base for the UpdateCallable structure that all derived VoxelGrid classes to use
    ///        when performing an update to their data with `std::visit`.
    /// @details This provides a default unsupported definition for all datatypes. A derived class
    ///          only needs to define `operator()` functions for the types which its derived
    ///          VoxelGrid explicitly supports.
    /// @note To prevent name hiding of this class's default `operator()` functions in a derived
    ///       VoxelGrid's UpdateCallable, the UpdateCallable body must include:
    ///           `using VoxelGrid::UpdateCallable::operator();`
    struct UpdateCallable
    {
        /// @brief Acquires temporary, shared ownership of a trace.
        /// @param ray_trace Trace to perform an update from.
        void acquireRayTrace(const std::shared_ptr<const Trace>& ray_trace)
        {
            this->ray_trace = ray_trace;
        }


        /// @brief Releases the VoxelGrid's reference to the trace.
        void releaseRayTrace()
        {
            this->ray_trace.reset();
        }


        /// @brief Parameter for the voxel update functions.
        std::shared_ptr<const Trace> ray_trace{nullptr};

        /// @brief A the error message if a type is not supported.
        static const std::string type_not_supported_message;


        // ************************************************************************************* //
        // *                               UNSUPPORTED DATATYPES                               * //
        // * These should be unreachable; the VoxelGrid constructor should ensure no invalid   * //
        // * vectors are used in this derived class. But for safety these are still defined to * //
        // * throw a DataVariantError error if they are ever reached.                                   * //
        // ************************************************************************************* //

        void operator()(std::vector<int8_t>&)   { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<int16_t>&)  { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<int32_t>&)  { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<int64_t>&)  { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<uint8_t>&)  { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<uint16_t>&) { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<uint32_t>&) { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<size_t>&)   { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<float>&)    { throw DataVariantError(this->type_not_supported_message); }
        void operator()(std::vector<double>&)   { throw DataVariantError(this->type_not_supported_message); }
    };


    // ***************************************************************************************** //
    // *                               PROTECTED CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Protected constructor to enforce use of derived classes.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    ///                   These `Grid::Properties` are utilized by all VoxelGrids.
    /// @param dist_min   Minimum trace update distance for this VoxelGrid.
    /// @param dist_max   Maximum trace update distance for this VoxelGrid.
    /// @param default_value  Initialization value for all voxels in this VoxelGrid.
    /// @param type_id        DataType enumeration to identify what the data type this Voxel
    ///                       Grid's data vector holds.
    /// @param valid_type_ids Bit field of data types this a derived VoxelGrid will accept.
    /// @throws DataVariantError if the DataType of `type_id` is not supported by this VoxelGrid.
    VoxelGrid(std::shared_ptr<const Grid::Properties> properties,
              const float& dist_min,
              const float& dist_max,
              const DataVariant& default_value,
              const DataType& type_id,
              const DataType& valid_type_ids = DataType::TYPE_ANY)
        : Grid(properties, this->validDataTypeID(type_id, valid_type_ids)),
          dist_min(std::min(dist_min, dist_max)),
          dist_max(std::max(dist_min, dist_max)),
          default_value(this->setDefaultValue(default_value))
    {
        const size_t n = this->properties->getNumVoxels();

        // Factory for the correct type_id of vector into the variant.
        if (this->type_id == DataType::INT8_T)
        {
            this->data = std::vector<int8_t>(n, std::get<int8_t>(this->default_value));
        }
        else if (this->type_id == DataType::INT16_T)
        {
            this->data = std::vector<int16_t>(n, std::get<int16_t>(this->default_value));
        }
        else if (this->type_id == DataType::INT32_T)
        {
            this->data = std::vector<int32_t>(n, std::get<int32_t>(this->default_value));
        }
        else if (this->type_id == DataType::INT64_T)
        {
            this->data = std::vector<int64_t>(n, std::get<int64_t>(this->default_value));
        }
        else if (this->type_id == DataType::UINT8_T)
        {
            this->data = std::vector<uint8_t>(n, std::get<uint8_t>(this->default_value));
        }
        else if (this->type_id == DataType::UINT16_T)
        {
            this->data = std::vector<uint16_t>(n, std::get<uint16_t>(this->default_value));
        }
        else if (this->type_id == DataType::UINT32_T)
        {
            this->data = std::vector<uint32_t>(n, std::get<uint32_t>(this->default_value));
        }
        else if (this->type_id == DataType::SIZE_T)
        {
            this->data = std::vector<size_t>(n, std::get<size_t>(this->default_value));
        }
        else if (this->type_id == DataType::FLOAT)
        {
            this->data = std::vector<float>(n, std::get<float>(this->default_value));
        }
        else if (this->type_id == DataType::DOUBLE)
        {
            this->data = std::vector<double>(n, std::get<double>(this->default_value));
        }
        else
        {
            throw DataVariantError::UnrecognizedEnumeration(this->type_id);
        }
    }


    /// @brief Writes the VoxelGrid's data vector to the provided HDF5 group.
    /// @param g_channel Group in for the opened HDF5 file.
    /// @param grid_type Name of the derived class.
    /// @throws std::runtime_error If there is a bad variant access.
    /// @throws DataVariantError If the VoxelGrid's `type_id` is not recognized.
    ///         (However, this error should be caught earlier in the VoxelGrid constructor.)
    /// @note This is virtual so VoxelGrid with multiple data channels may specifically handle
    ///       their channels. But most derived VoxelGrids may uses this method.
    virtual void save(HighFive::Group& g_channel, const std::string& grid_type)
    {
        try
        {
            if (this->type_id == DataType::INT8_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<int8_t>>(this->data));
            }
            else if (this->type_id == DataType::INT16_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<int16_t>>(this->data));
            }
            else if (this->type_id == DataType::INT32_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<int32_t>>(this->data));
            }
            else if (this->type_id == DataType::INT64_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<int64_t>>(this->data));
            }
            else if (this->type_id == DataType::UINT8_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<uint8_t>>(this->data));
            }
            else if (this->type_id == DataType::UINT16_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<uint16_t>>(this->data));
            }
            else if (this->type_id == DataType::UINT32_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<uint32_t>>(this->data));
            }
            else if (this->type_id == DataType::SIZE_T)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<size_t>>(this->data));
            }
            else if (this->type_id == DataType::FLOAT)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<float>>(this->data));
            }
            else if (this->type_id == DataType::DOUBLE)
            {
                g_channel.createDataSet(grid_type, std::get<std::vector<double>>(this->data));
            }
            else
            {
                DataVariantError::UnrecognizedEnumeration(this->type_id);
            }
        }
        catch (const std::bad_variant_access&)
        {
            throw std::runtime_error("Bad variant access: Cannot save Grid's Data vector to HDF5 file.");
        }
    }



    /// @brief Adds this VoxelGrid's data to the XDMF file provided by the Reconstruction class.
    /// @param file An opened file stream.
    /// @param hdf5_fname File name (not the full path) of the HDF5 file that this XDMF relates to.
    /// @param grid_name  The dictionary map name of this grid.
    /// @param grid_type  The type name of this grid.
    /// @note This is virtual so VoxelGrid with multiple data channels may specifically handle
    ///       their channels. But most derived VoxelGrids may uses this method.
    virtual void addToXDMF(std::ofstream& file,          const std::string& hdf5_fname,
                           const std::string& grid_name, const std::string& grid_type) const
    {
        utilities::XDMF::writeVoxelGridAttribute(
            file,
            grid_name,
            utilities::XDMF::makeDataPath(hdf5_fname, FS_HDF5_RECONSTRUCTION_GROUP, grid_name, grid_type),
            getNumberTypeXDMF(this->type_id),
            getNumberPrecisionXDMF(this->type_id),
            this->properties->getNumVoxels()
        );
    }



    // ***************************************************************************************** //
    // *                               PROTECTED CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief Container variant which may store any number of numeric types.
    /// @note If a derived class has multiple data channels this should be used for the main
    ///       data a user might access or modify.
    VectorVariant data;


    /// @brief Container of the same shape as `VoxelGrid::data` in any derived grid, but this stores
    ///        a boolean flag for if a voxel was intersected by the postive region of a ray at least.
    /// @note  This is a view of data managed by `data::Reconstruction`.
    std::shared_ptr<const std::vector<bool>> data_seen{nullptr};

private:
    // ***************************************************************************************** //
    // *                                PRIVATE CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Helper function to ensure the DataVariant passed into the VoxelGrid constructor
    ///        is set to the correct data type before it is used to initialize the data vector.
    /// @param x The input to cast.
    /// @return A DataVariant that has the input's value cast to the correct data type.
    /// @throws DataVariantError If the VoxelGrid's `type_id` is not recognized.
    ///         (However, this error should be caught earlier in the VoxelGrid constructor.)
    DataVariant setDefaultValue(const DataVariant& x)
    {
        DataVariant res;
        if (this->type_id == DataType::INT8_T)
        {
            auto call_cast = [&res](auto&& x){ res = static_cast<int8_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::INT16_T)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<int16_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::INT32_T)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<int32_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::INT64_T)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<int64_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::UINT8_T)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<uint8_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::UINT16_T)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<uint16_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::UINT32_T)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<uint32_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::SIZE_T)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<size_t>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::FLOAT)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<float>(x); };
            std::visit(call_cast, x);
        }
        else if (this->type_id == DataType::DOUBLE)
        {
            auto call_cast = [&res](auto&& x) { res = static_cast<double>(x); };
            std::visit(call_cast, x);
        }
        else
        {
            DataVariantError::UnrecognizedEnumeration(this->type_id);
        }
        return res;
    }



    // ***************************************************************************************** //
    // *                            STATIC PRIVATE CLASS METHODS                               * //
    // ***************************************************************************************** //


    /// @brief Helper to verify that a derived class can only be constructed with the data types
    ///        it supports.
    /// @param requested ID value for the requested data type.
    /// @param should_be ID value for the valid data types.
    /// @return Returns the input if it is valid
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    /// @throws DataVariantError if the DataType is a type checking bitfield and does
    ///         not actually represent a data type.
    static DataType validDataTypeID(const DataType& requested, const DataType& should_be)
    {
        if ( !(requested & DataType::TYPE_NOT_A_TYPE_CHECK) )
        {
            throw DataVariantError::AttemptedUseOfTypeCheckingDataType(dataTypeToString(requested));
        }
        if (requested & should_be)
        {
            return requested;
        }
        throw DataVariantError::VoxelGridDoesNotSupport(dataTypeToString(requested), dataTypeToString(should_be));
    }
};


// Provide definition for the static variable declared in Callable base class.
const std::string VoxelGrid::UpdateCallable::type_not_supported_message =
    std::string("A VoxelGrid's Update Callable encountered a vector variant of an unsupported data type. "
                "PLEASE CHECK WHAT YOU HAVE DONE: THIS EXCEPTION SHOULD NEVER BE REACHED.");

/// @brief String for the class name.
const std::string VoxelGrid::type_name = "VoxelGrid";

/// @brief Default values for common distances or initialization values used by derived VoxelGrids.
const float VoxelGrid::default_zero     = 0.0f,
            VoxelGrid::default_infinity = INFINITY,
            VoxelGrid::default_d_min      = -0.2f,
            VoxelGrid::default_d_max      =  0.2f;


/// @brief ArgParser key for the derived VoxelGrid type to create.
const std::string VoxelGrid::parse_type = "--type";

/// @brief ArgParser key for the minimum update distance.
const std::string VoxelGrid::parse_d_min = "--d-min";

/// @brief ArgParser key for the maximum update distance.
const std::string VoxelGrid::parse_d_max = "--d-max";

/// @brief ArgParser key for the default (and initial) values of voxels.
const std::string VoxelGrid::parse_default = "--default";

/// @brief ArgParser key for the data type of the voxels.
const std::string VoxelGrid::parse_dtype = "--dtype";


} // namespace data
} // namespace forge_scan


#endif // DEMOS_CPP_FORGE_SCAN_RECONSTRUCTION_VOXEL_GRID_HPP
