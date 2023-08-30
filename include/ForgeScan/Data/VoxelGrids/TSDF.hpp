#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_TSDF_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_TSDF_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"
#include "ForgeScan/Utilities/Math.hpp"


namespace forge_scan {
namespace data {


/// @brief Represents a truncated signed-distance function (TSDF).
/// @note Uses a minimum magnitude strategy to update the voxel's distance.
/// @note Supports `float` and `double` data types only.
class TSDF : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to a TSDF Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param parser Arg Parser with arguments to construct an TSDF Grid from.
    /// @return Shared pointer to a TSDF Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                        const utilities::ArgParser& parser)
    {
        return create(properties, parser.getCmdOption<float>("--dist-min", -0.2),
                                  parser.getCmdOption<float>("--dist-max",  0.2),
                                  stringToDataType(parser.getCmdOption("--data-type"), DataType::FLOAT));
    }


    /// @brief Constructor for a shared pointer to a TSDF Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param dist_min   Minimum update distance. Default -0.2.
    /// @param dist_max   Maximum update distance. Default +0.2.
    /// @param type_id Datatype for the Grid. Default is float.
    /// @return Shared pointer to a TSDF Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                        const float& dist_min = -0.2,
                                        const float& dist_max =  0.2,
                                        const DataType& type_id = DataType::FLOAT)
    {
        return std::shared_ptr<TSDF>(new TSDF(properties, dist_min, dist_max, type_id));
    }


    /// @brief Returns the class type name for the Voxel Grid.
    const std::string& getTypeName() const override final
    {
        static const std::string name = "TSDF";
        return name;
    }


    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(std::shared_ptr<const trace> ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }


private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param dist_min   Minimum trace update distance for this Voxel Grid.
    /// @param dist_max   Maximum trace update distance for this Voxel Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    explicit TSDF(const std::shared_ptr<const Grid::Properties>& properties,
                  const float& dist_min,
                  const float& dist_max,
                  const DataType& type_id)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    NEGATIVE_INFINITY,
                    type_id,
                    DataType::TYPE_FLOATING_POINT),
          update_callable(*this)
    {

    }


    /// @brief Subclass provides update functions for each supported DataType/VectorVariant of
    ///        the data vector. 
    struct UpdateCallable : public VoxelGrid::Callable
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
        UpdateCallable(TSDF& caller)
            : caller(caller)
        {

        }


        /// @brief Reference to the specific derived class calling this object.
        TSDF& caller;
    };


    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This musts be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};



} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP
