#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"



namespace forge_scan {
namespace data {


/// @brief Counts how many times the voxel has been updated.
/// @note Rollover of integer types may occur if the type is too small.
class CountUpdates : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to a CountUpdates VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param parser ArgParser with arguments to construct an CountUpdates Grid from.
    /// @return Shared pointer to a CountUpdates Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<CountUpdates> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const utilities::ArgParser& parser)
    {
        return create(properties, parser.get<float>(VoxelGrid::parse_d_min,   VoxelGrid::default_zero),
                                  parser.get<float>(VoxelGrid::parse_d_max,   VoxelGrid::default_infinity),
                                  parser.get<float>(VoxelGrid::parse_default, VoxelGrid::default_zero),
                                  stringToDataType(parser.get(VoxelGrid::parse_dtype), DataType::UINT32_T));
    }


    /// @brief Constructor for a shared pointer to a CountUpdates VoxelGrid.
    /// @param properties Shared, constant properties for the reconstruction.
    /// @param dist_min   Minimum update distance. Default 0.
    /// @param dist_max   Maximum update distance. Default positive infinity.
    /// @param default_value Value to initialize the Grid to. Default 0.
    /// @param type_id       Datatype for the Grid. Default is unsigned 32-bit integer.
    /// @return Shared pointer to a CountUpdates Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<CountUpdates> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const float& dist_min = 0,
                                               const float& dist_max = INFINITY,
                                               const DataVariant& default_value = 0,
                                               const DataType& type_id = DataType::UINT32_T)
    {
        return std::shared_ptr<CountUpdates>(new CountUpdates(properties, dist_min, dist_max, default_value, type_id));
    }


    /// @return Help message for constructing a CountUpdates VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: CountUpdates help message";
    }


    /// @brief Returns the class type name for the VoxelGrid.
    const std::string& getTypeName() const override final
    {
        return CountUpdates::type_name;
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
    /// @param properties Shared, constant properties for the reconstruction.
    /// @param dist_min   Minimum update distance.
    /// @param dist_max   Maximum update distance.
    /// @param default_value Value to initialize the Grid to
    /// @param type_id       Datatype for the Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    explicit CountUpdates(const std::shared_ptr<const Grid::Properties>& properties,
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
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<int16_t>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<int32_t>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<int64_t>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<uint8_t>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<uint16_t>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<uint32_t>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<size_t>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<float>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        void operator()(std::vector<double>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                ++vector[iter->i];
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(CountUpdates& caller)
            : caller(caller)
        {

        }


        /// @brief Reference to the specific derived class calling this object.
        CountUpdates& caller;
    };


    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This must be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};


/// @brief String for the class name.
const std::string CountUpdates::type_name = "CountUpdates";


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP
