#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"



namespace forge_scan {
namespace data {


/// @brief Counts how many times the voxel has been updated.
/// @note Rollover of integer types may occur if the type is too small.
class UpdateCount : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to a UpdateCount Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param parser Arg Parser with arguments to construct an UpdateCount Grid from.
    /// @return Shared pointer to a UpdateCount Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<UpdateCount> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const utilities::ArgParser& parser)
    {
        return create(properties, parser.getCmdOption<float>("--dist-min", 0),
                                  parser.getCmdOption<float>("--dist-max", INFINITY),
                                  parser.getCmdOption<float>("--default-val", 0),
                                  stringToDataType(parser.getCmdOption("--data-type"), DataType::UINT32_T));
    }


    /// @brief Constructor for a shared pointer to a UpdateCount VoxelGrid.
    /// @param properties Shared, constant properties for the reconstruction.
    /// @param dist_min   Minimum update distance. Default 0.
    /// @param dist_max   Maximum update distance. Default positive infinity.
    /// @param default_value Value to initialize the Grid to. Default 0.
    /// @param type_id       Datatype for the Grid. Default is unsigned 32-bit integer.
    /// @return Shared pointer to a UpdateCount Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<UpdateCount> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const float& dist_min = 0,
                                               const float& dist_max = INFINITY,
                                               const DataVariant& default_value = 0,
                                               const DataType& type_id = DataType::UINT32_T)
    {
        return std::shared_ptr<UpdateCount>(new UpdateCount(properties, dist_min, dist_max, default_value, type_id));
    }


    /// @brief Returns the class type name for the Voxel Grid.
    const std::string& getTypeName() const override final
    {
        static const std::string name = "UpdateCount";
        return name;
    }


    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(const std::shared_ptr<const Trace>& ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }


private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant properties for the reconstruction.
    /// @param dist_min   Minimum update distance.
    /// @param dist_max   Maximum update distance.
    /// @param default_value Value to initialize the Grid to
    /// @param type_id       Datatype for the Grid.
    /// @throws std::invalid_argument if the DataType is not supported by this VoxelGrid.
    explicit UpdateCount(const std::shared_ptr<const Grid::Properties>& properties,
                         const float& dist_min,
                         const float& dist_max,
                         const DataVariant& default_value,
                         const DataType& type_id)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    default_value,
                    type_id,
                    DataType::TYPE_ANY),
          update_callable(*this)
    {

    }


    /// @brief Subclass provides update functions for each supported DataType/VectorVariant of
    ///        the data vector. 
    struct UpdateCallable : public VoxelGrid::UpdateCallable
    {
        using VoxelGrid::UpdateCallable::operator();

        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<int8_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<int16_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<int32_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<int64_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<uint8_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<uint16_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<uint32_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<size_t>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<float>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<double>& vector)
        {
            Trace::const_iterator iter = ray_trace_helpers::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->d > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                ++vector[iter->i];
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(UpdateCount& caller)
            : caller(caller)
        {

        }


        /// @brief Reference to the specific derived class calling this object.
        UpdateCount& caller;
    };


    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This must be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP
