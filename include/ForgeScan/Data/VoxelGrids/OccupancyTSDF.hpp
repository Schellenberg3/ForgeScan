#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_OCCUPANCY_TSDF_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_OCCUPANCY_TSDF_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"
#include "ForgeScan/Utilities/Math.hpp"


namespace forge_scan {
namespace data {


/// @brief Represents a truncated signed-distance function (Occupancy TSDF).
/// @note Uses a minimum magnitude strategy to update the voxel's distance.
/// @note Supports `float` and `double` data types only.
class OccupancyTSDF : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to a Occupancy Occupancy TSDF Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param parser Arg Parser with arguments to construct an Occupancy Occupancy TSDF Grid from.
    /// @return Shared pointer to a Occupancy Occupancy TSDF Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<OccupancyTSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                                 const utilities::ArgParser& parser)
    {
        return create(properties, parser.getCmdOption<float>("--dist-min", -0.2),
                                  parser.getCmdOption<float>("--dist-max",  0.2),
                                  stringToDataType(parser.getCmdOption("--data-type"), DataType::FLOAT));
    }


    /// @brief Constructor for a shared pointer to a Occupancy Occupancy TSDF Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param dist_min   Minimum update distance. Default -0.2.
    /// @param dist_max   Maximum update distance. Default +0.2.
    /// @param type_id Datatype for the Grid. Default is float.
    /// @return Shared pointer to a Occupancy Occupancy TSDF Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<OccupancyTSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                                 const float& dist_min = -0.2,
                                                 const float& dist_max =  0.2,
                                                 const DataType& type_id = DataType::FLOAT)
    {
        return std::shared_ptr<OccupancyTSDF>(new OccupancyTSDF(properties, dist_min, dist_max, type_id));
    }


    /// @brief Returns the class type name for the Voxel Grid.
    const std::string& getTypeName() const override final
    {
        static const std::string name = "OccupancyTSDF";
        return name;
    }


    /// @brief Accessor for `metrics::ground_truth::ExperimentOccupancy` in 
    ///       `metrics::OccupancyConfusion`
    /// @return Read-only reference to the Occupancy data vector.  
    const std::vector<uint8_t>& getOccupancyData() const
    {
        return this->data_occupancy;
    }


private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param dist_min   Minimum trace update distance for this Voxel Grid.
    /// @param dist_max   Maximum trace update distance for this Voxel Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    explicit OccupancyTSDF(const std::shared_ptr<const Grid::Properties>& properties,
                           const float& dist_min,
                           const float& dist_max,
                           const DataType& type_id)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    NEGATIVE_INFINITY,
                    type_id,
                    DataType::TYPE_FLOATING_POINT),
          data_occupancy(std::vector<uint8_t>(this->properties->getNumVoxels(),
                                              VoxelOccupancy::UNKNOWN)),
          update_callable(*this)
    {

    }


    /// @brief Writes the Voxel Grid's data vector to the provided HDF5 group.
    /// @param g_channel Group in for the opened HDF5 file.
    /// @param grid_type Name of the derived class.
    /// @throws `std::runtime_error` If there is a bad variant access.
    ///         (However, this error should be caught earlier in the Voxel Grid constructor.)
    /// @throws `std::runtime_error` If the Voxel Grid's `type_id` is not recognized or supported.
    ///         (However, this error should be caught earlier in the Voxel Grid constructor.)
    /// @note  This override allows the class to add information about the multiple data vectors it has.
    void save(HighFive::Group& g_channel, const std::string& grid_type) const override final
    {
        try
        {
            if (this->type_id == DataType::FLOAT)
            {
                g_channel.createDataSet(grid_type + "_TSDF", std::get<std::vector<float>>(this->data));
                g_channel.createDataSet(grid_type + "_Occupancy", this->data_occupancy);
            }
            else if (this->type_id == DataType::DOUBLE)
            {
                g_channel.createDataSet(grid_type + "_TSDF", std::get<std::vector<double>>(this->data));
                g_channel.createDataSet(grid_type + "_Occupancy", this->data_occupancy);
            }
            else
            {
                throw std::runtime_error("Cannot save Grid's Data vector to HDF5 file: Unsupported data type_id.");
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
    /// @note  This override allows the class to add information about the multiple data vectors it has.
    void addToXDMF(std::ofstream& file,          const std::string& hdf5_fname,
                   const std::string& grid_name, const std::string& grid_type) const override final
    {
        // Write TSDF data
        utilities::XDMF::writeVoxelGridAttribute(
            file,
            grid_name + "_TSDF",
            utilities::XDMF::makeDataPath(hdf5_fname, FS_HDF5_RECONSTRUCTION_GROUP, grid_name, grid_type + "_TSDF"),
            getNumberTypeXDMF(this->type_id),
            getNumberPrecisionXDMF(this->type_id),
            this->properties->getNumVoxels()
        );

        // Write occupancy data
        utilities::XDMF::writeVoxelGridAttribute(
            file,
            grid_name + "_Occupancy",
            utilities::XDMF::makeDataPath(hdf5_fname, FS_HDF5_RECONSTRUCTION_GROUP, grid_name, grid_type + "_Occupancy"),
            getNumberTypeXDMF(DataType::UINT8_T),
            getNumberPrecisionXDMF(DataType::UINT8_T),
            this->properties->getNumVoxels()
        );
    }


    /// @brief Subclass provides update functions for each supported DataType/VectorVariant of
    ///        the data vector. 
    struct UpdateCallable
    {
        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<float>& vector) const
        {
            trace::const_iterator iter = ray_trace::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->second > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                this->caller.data_occupancy[iter->first] = iter->second <= 0 ? VoxelOccupancy::OCCUPIED : VoxelOccupancy::FREE;

                if(utilities::math::is_greater_in_magnitude(vector[iter->first], iter->second))
                {
                    vector[iter->first] = iter->second;
                }
            }
        }


        void operator()(std::vector<double>& vector) const
        {
            trace::const_iterator iter = ray_trace::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->second > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                this->caller.data_occupancy[iter->first] = iter->second <= 0 ? VoxelOccupancy::OCCUPIED : VoxelOccupancy::FREE;

                if(utilities::math::is_greater_in_magnitude(vector[iter->first], static_cast<double>(iter->second)))
                {
                    vector[iter->first] = iter->second;
                }
            }
        }


        // ************************************************************************************* //
        // *                               UNSUPPORTED DATATYPES                               * //
        // * These should be unreachable; the VoxelGrid constructor should ensure no invalid   * //
        // * vectors are used in this derived class. But for safety these are still defined to * //
        // * throw a runtime error if they are ever reached.                                   * //
        // ************************************************************************************* //

        void operator()(std::vector<int8_t>&)   const { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<int16_t>&)  const { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<int32_t>&)  const { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<int64_t>&)  const { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<uint8_t>&)  const { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<uint16_t>&) const { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<uint32_t>&) const { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<size_t>&)   const { throw std::runtime_error(this->type_not_supported_message); }



        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(OccupancyTSDF& caller)
            : caller(caller)
        {

        }


        /// @brief Acquires temporary shared ownership of a trace.
        /// @param ray_trace Trace to perform update from.
        void acquireRayTrace(std::shared_ptr<const trace> ray_trace)
        {
            this->ray_trace = ray_trace;
        }


        /// @brief Releases the VoxelGrid's reference to the trace.
        void releaseRayTrace()
        {
            this->ray_trace.reset();
        }


        /// @brief Parameter for the voxel update functions.
        std::shared_ptr<const trace> ray_trace{nullptr};

        /// @brief Reference to the specific derived class calling this object.
        OccupancyTSDF& caller;

        /// @brief A message for the error message if a type is not supported.
        const std::string type_not_supported_message = "Occupancy TSDF only supports voxel vectors of float and double types. "
                                                       "PLEASE CHECK WHAT YOU HAVE DONE: THIS EXCEPTION SHOULD NEVER BE REACHED.";
    };


public:
    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(std::shared_ptr<const trace> ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }


private:
    /// @brief Stores the occupancy data that the grid uses.
    std::vector<uint8_t> data_occupancy; 

    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This must be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};



} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_OCCUPANCY_TSDF_HPP
