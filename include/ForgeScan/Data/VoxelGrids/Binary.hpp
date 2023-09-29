#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"


namespace forge_scan {
namespace data {


/// @brief Tracks binary occupancy values for each voxel. The whole VoxelGrid begins as "occupied"
///        and are updated to be "free" rays travel through them.
class Binary : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to an Binary VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param parser ArgParser with arguments to construct an Binary Grid from.
    /// @return Shared pointer to a Binary Grid.
    static std::shared_ptr<Binary> create(const std::shared_ptr<const Grid::Properties>& properties,
                                             const utilities::ArgParser& parser)
    {
        return create(properties, parser.get<float>(VoxelGrid::parse_d_min, VoxelGrid::default_zero),
                                  parser.get<float>(VoxelGrid::parse_d_max, VoxelGrid::default_infinity));
    }


    /// @brief Constructor for a shared pointer to an Binary VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min    Minimum update distance. Default 0.
    /// @param dist_max    Maximum update distance. Default infinity.
    /// @return Shared pointer to an Binary Grid.
    static std::shared_ptr<Binary> create(const std::shared_ptr<const Grid::Properties>& properties,
                                             const float& dist_min = 0,
                                             const float& dist_max = INFINITY)
    {
        return std::shared_ptr<Binary>(new Binary(properties, dist_min, dist_max));
    }


    /// @return Help message for constructing a Binary VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: Binary help message";
    }

    /// @brief Returns the class type name for the VoxelGrid.
    const std::string& getTypeName() const override final
    {
        static const std::string name = "Binary";
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
    void update(const std::shared_ptr<const Trace>& ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }

    static const std::string type_name;

private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min   Minimum trace update distance for this VoxelGrid.
    /// @param dist_max   Maximum trace update distance for this VoxelGrid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    explicit Binary(const std::shared_ptr<const Grid::Properties>& properties,
                       const float& dist_min,
                       const float& dist_max)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    VoxelOccupancy::UNSEEN,
                    DataType::UINT8_T,
                    DataType::UINT8_T),
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


        void operator()(std::vector<uint8_t>& vector)
        {
            Trace::const_iterator iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last_occ  = this->ray_trace->first_above(0, iter);
            const Trace::const_iterator last_free = this->ray_trace->first_above(this->caller.dist_max, last_occ);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last_occ; ++iter)
            {
                vector[iter->i] = VoxelOccupancy::OCCLUDED;
            }
            for ( ; iter != last_free; ++iter)
            {
                vector[iter->i] = VoxelOccupancy::FREE;
            }
            if (this->ray_trace->hasSensed())
            {
                vector[last_occ->i] = VoxelOccupancy::OCCUPIED;
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(Binary& caller)
            : caller(caller)
        {

        }


        /// @brief Reference to the specific derived class calling this object.
        Binary& caller;
    };


    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This musts be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};


/// @brief String for the class name.
const std::string Binary::type_name = "Binary";


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_HPP
