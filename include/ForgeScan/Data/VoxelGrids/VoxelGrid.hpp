#ifndef DEMOS_CPP_FORGE_SCAN_RECONSTRUCTION_VOXEL_GRID_HPP
#define DEMOS_CPP_FORGE_SCAN_RECONSTRUCTION_VOXEL_GRID_HPP

#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>

#include "ForgeScan/Common/Definitions.hpp"
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


/// @brief A base class for a Voxel Grid of various data types. Derived classes define what datatypes they accept
///        and implement strategies for updating with new data.
class VoxelGrid : public Grid
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @brief Required to call the save method.
    friend class Reconstruction;


public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Calculates how much space the data vector is using.
    /// @param size_bytes[out]     Number of bytes used by the vector.
    /// @param capacity_bytes[out] Number of bytes used by the vector, including capacity.
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
    /// @param size_bytes[out]     Number of megabytes used by the vector.
    /// @param capacity_bytes[out] Number of megabytes used by the vector, including capacity.
    virtual void getDataMemoryUsage(float& size_mb, float& capacity_mb) const
    {
        size_t size_bytes, capacity_bytes;
        this->getDataMemoryUsage(size_bytes, capacity_bytes);
        size_mb     = utilities::memory_use::byte_to_megabytes(size_bytes);
        capacity_mb = utilities::memory_use::byte_to_megabytes(capacity_bytes);
    }


    /// @brief Gets a constant reference to the VectorVariant data the Voxel Grid stores.
    /// @return Read-only reference to the data.
    const VectorVariant& getData() const
    {
        return this->data;
    }



    // ***************************************************************************************** //
    // *                             PUBLIC PURE VIRTUAL METHODS                               * //
    // ***************************************************************************************** //

    /// @brief Returns the class type name for the Voxel Grid.
    virtual const std::string& getTypeName() const = 0;


    /// @brief Updates the Voxel Grid with new information along a ray.
    /// @param ray_trace A trace to update the Voxel Grid along. 
    virtual void update(std::shared_ptr<const trace> ray_trace) = 0;



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //

    
    /// @brief The minimum and maximum distance along a trace that the Voxel Grid will perform
    ///        updates at. A minimum of 0 means the Grid will start at the sensed point. A maximum
    ///        of infinity means the Grid will update until the last voxel on the trace (the ray's
    ///        origin or the Grid's boundary). Negative values are valid too.
    ///        The VoxelGrid constructor guarantees that dist_min <= dist_max.
    const float dist_min, dist_max;

    /// @brief Variant type which holds the default value for the Voxel Grid.
    ///        All elements are initialized to this value.
    DataVariant default_value;


protected:
    // ***************************************************************************************** //
    // *                               PROTECTED CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Protected constructor to enforce use of derived classes.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    ///                   These Grid Properties are utilized by all Voxel Grids.
    /// @param dist_min   Minimum trace update distance for this Voxel Grid.
    /// @param dist_max   Maximum trace update distance for this Voxel Grid.
    /// @param default_value  Initialization value for all voxels in this Voxel Grid.
    /// @param type_id        DataType enumeration to identify what the data type this Voxel
    ///                       Grid's data vector holds.
    /// @param valid_type_ids Bit field of data types this a derived Voxel Grid will accept.
    /// @throws `std::invalid_argument` if the DataType of `type_id` is not supported by this Voxel Grid.
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

        /* Factory for the correct type_id of vector into the variant. */
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
            throw std::invalid_argument("Unrecognized data type_id: " + std::to_string(this->type_id) + ". Cannot set up the container's vector.");
        }
    }


    /// @brief Writes the Voxel Grid's data vector to the provided HDF5 group.
    /// @param g_channel Group in for the opened HDF5 file.
    /// @param grid_type Name of the derived class.
    /// @throws `std::runtime_error` If there is a bad variant access.
    /// @throws `std::runtime_error` If the Voxel Grid's `type_id` is not recognized.
    ///         (However, this error should be caught earlier in the Voxel Grid constructor.)
    /// @note This is virtual so Voxel Grid with multiple data channels may specifically handle
    ///       their channels. But most derived Voxel Grids may uses this method.
    virtual void save(HighFive::Group& g_channel, const std::string& grid_type) const
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
                throw std::runtime_error("Cannot save Grid's Data vector to HDF5 file: Unrecognized data type_id.");
            }
        }
        catch (const std::bad_variant_access& e)
        {
            throw std::runtime_error("Bad variant access: Cannot save Grid's Data vector to HDF5 file.");
        }
    }



    /// @brief Adds this Voxel Grid's data to the XDMF file provided by the Reconstruction class. 
    /// @param file An opened file stream.
    /// @param hdf5_fname File name (not the full path) of the HDF5 file that this XDMF relates to.
    /// @param grid_name  The dictionary map name of this grid. 
    /// @param grid_type  The type name of this grid.
    /// @note This is virtual so Voxel Grid with multiple data channels may specifically handle
    ///       their channels. But most derived Voxel Grids may uses this method.
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


private:
    // ***************************************************************************************** //
    // *                                PRIVATE CLASS METHODS                                  * //
    // ***************************************************************************************** //
 

    /// @brief Helper function to ensure the DataVariant passed into the Voxel Grid constructor
    ///        is set to the correct data type before it is used to initialize the data vector.
    /// @param x The input to cast.
    /// @return A DataVariant that has the input's value cast to the correct data type.
    /// @throws `std::runtime_error` If the Voxel Grid's `type_id` is not recognized.
    ///         (However, this error should be caught earlier in the Voxel Grid constructor.)
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
            throw std::invalid_argument("Unrecognized data type_id: " + std::to_string(this->type_id)
                                        + ". Cannot set up the container's default value.");
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
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    /// @throws std::invalid_argument if the DataType is a type checking bitfield and does
    ///         not actually represent a data type.
    static DataType validDataTypeID(const DataType& requested, const DataType& should_be)
    {
        if ( !(requested & DataType::TYPE_NOT_A_TYPE_CHECK) )
        {
            throw std::invalid_argument("The requested VoxelGrid can only accept " +
                                        DataTypeToString.at(should_be) + " but the type " +
                                        DataTypeToString.at(requested) + " was requested. "
                                        "Cannot use type checking enumeration values as data type IDs.");
        }
        if (requested & should_be)
        {
            return requested;
        }
        throw std::invalid_argument("The requested VoxelGrid can only accept " +
                                    DataTypeToString.at(should_be) + " but the type " +
                                    DataTypeToString.at(requested) + " was requested.");
    }
};


} // namespace data
} // namespace forge_scan


#endif // DEMOS_CPP_FORGE_SCAN_RECONSTRUCTION_VOXEL_GRID_HPP
