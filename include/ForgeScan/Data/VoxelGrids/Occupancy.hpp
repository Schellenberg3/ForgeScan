#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_OCCUPANCY_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_OCCUPANCY_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"


namespace forge_scan {
namespace data {


/// @brief Tracks binary occupancy values for each voxel. The whole Voxel Grid begins as "occupied"
///        and are updated to be "free" rays travel through them. 
class Occupancy : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to an Occupancy Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param parser Arg Parser with arguments to construct an Occupancy Grid from.
    /// @return Shared pointer to a Occupancy Grid.
    static std::shared_ptr<Occupancy> create(const std::shared_ptr<const Grid::Properties>& properties,
                                             const utilities::ArgParser& parser)
    {
        return create(properties, parser.getCmdOption<float>("--dist-min", 0),
                                  parser.getCmdOption<float>("--dist-max", INFINITY));
    }


    /// @brief Constructor for a shared pointer to an Occupancy Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param dist_min    Minimum update distance. Default 0.
    /// @param dist_max    Maximum update distance. Default infinity.
    /// @return Shared pointer to an Occupancy Grid.
    static std::shared_ptr<Occupancy> create(const std::shared_ptr<const Grid::Properties>& properties,
                                             const float& dist_min = 0,
                                             const float& dist_max = INFINITY)
    {
        return std::shared_ptr<Occupancy>(new Occupancy(properties, dist_min, dist_max));
    }


    /// @brief Returns the class type name for the Voxel Grid.
    const std::string& getTypeName() const override final
    {
        static const std::string name = "Occupancy";
        return name;
    }

    /// @brief Accessor for `metrics::ground_truth::ExperimentOccupancy` in 
    ///       `metrics::OccupancyConfusion`
    /// @return Read-only reference to the Occupancy data vector.  
    const std::vector<uint8_t>& getOccupancyData() const
    {
        return std::get<std::vector<uint8_t>>(this->data);
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
    ///         (This should not happen.)
    explicit Occupancy(const std::shared_ptr<const Grid::Properties>& properties,
                       const float& dist_min,
                       const float& dist_max)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    VoxelOccupancy::OCCUPIED,
                    DataType::UINT8_T,
                    DataType::UINT8_T),
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
    

        void operator()(std::vector<uint8_t>& vector)
        {
            trace::const_iterator iter = ray_trace::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end() || iter->second > this->caller.dist_max)
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                vector[iter->first] = iter->second <= 0 ? VoxelOccupancy::OCCUPIED : VoxelOccupancy::FREE;
            }
        }



        // ************************************************************************************* //
        // *                               UNSUPPORTED DATATYPES                               * //
        // * These should be unreachable; the VoxelGrid constructor should ensure no invalid   * //
        // * vectors are used in this derived class. But for safety these are still defined to * //
        // * throw a runtime error if they are ever reached.                                   * //
        // ************************************************************************************* //


        void operator()(std::vector<int8_t>&)   { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<int16_t>&)  { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<int32_t>&)  { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<int64_t>&)  { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<uint16_t>&) { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<uint32_t>&) { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<size_t>&)   { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<float>&)    { throw std::runtime_error(this->type_not_supported_message); }
        void operator()(std::vector<double>&)   { throw std::runtime_error(this->type_not_supported_message); }



        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(Occupancy& caller)
            : caller(caller)
        {

        }


        /// @brief Reference to the specific derived class calling this object.
        Occupancy& caller;
    };


    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This musts be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_OCCUPANCY_HPP
