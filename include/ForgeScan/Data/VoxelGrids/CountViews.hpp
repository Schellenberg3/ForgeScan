#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_COUNT_VIEWS_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_COUNT_VIEWS_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"



namespace forge_scan {
namespace data {


/// @brief Counts how many times the voxel has been viewed.
/// @details This increments the count for a voxel only once group of updates, rather than the
///          CountUpdates class which increments for each ray traced.
/// @note Rollover of integer types may occur if the type is too small.
class CountViews : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to a CountViews VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param parser ArgParser with arguments to construct an CountViews Grid from.
    /// @return Shared pointer to a CountViews Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<CountViews> create(const std::shared_ptr<const Grid::Properties>& properties,
                                              const utilities::ArgParser& parser)
    {
        return create(properties, stringToDataType(parser.get(VoxelGrid::parse_dtype), DataType::SIZE_T));
    }


    /// @brief Constructor for a shared pointer to a CountViews VoxelGrid.
    /// @param properties Shared, constant properties for the reconstruction.
    /// @param type_id    Datatype for the Grid. Default is unsigned 64-bit integer (size_t).
    /// @return Shared pointer to a CountViews Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<CountViews> create(const std::shared_ptr<const Grid::Properties>& properties,
                                              const DataType& type_id = DataType::SIZE_T)
    {
        return std::shared_ptr<CountViews>(new CountViews(properties, type_id));
    }


    /// @return Help message for constructing a CountViews VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: CountViews help message";
    }


    /// @brief Returns the class type name for the VoxelGrid.
    const std::string& getTypeName() const override final
    {
        return CountViews::type_name;
    }


    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(const std::shared_ptr<const Trace>& ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }


    /// @brief Performs post-update processing on the Grid.
    void postUpdate() override final
    {
        std::visit(this->post_update_callable, this->data);
    }


    /// @brief Returns how many voxels were viewed in the last update.
    /// @return Count of viewed voxels.
    size_t getViewedCount() const
    {
        return this->viewed_count;
    }


    /// @brief Returns how many voxels were occluded in the last update.
    /// @return Count of occluded voxels. Ones on, but behind, at a negative distance in ray.
    size_t getOccludedCount() const
    {
        return this->occluded_count;
    }


    /// @brief Returns how many voxels were unseen in the last update.
    /// @return Count of unseen voxels. (Neither viewed nor occluded.)
    size_t getUnseen() const
    {
        return this->unseen_count;
    }

    static const std::string type_name;

private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant properties for the reconstruction.
    /// @param default_value Value to initialize the Grid to
    /// @param type_id       Datatype for the Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    explicit CountViews(const std::shared_ptr<const Grid::Properties>& properties,
                        const DataType& type_id)
        : VoxelGrid(properties,
                    NEGATIVE_INFINITY,
                    INFINITY,
                    0,
                    type_id,
                    DataType::TYPE_UNSIGNED_INT),
          update_callable(*this),
          post_update_callable(*this)
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
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for (auto iter = this->ray_trace->begin(); iter != this->ray_trace->end(); ++iter)
            {
                const bool on_positive_ray = iter->d > 0.0f;
                vector[iter->i] |=  (on_positive_ray * u8_viewed) +
                                   (!on_positive_ray * u8_occluded);
            }
        }


        void operator()(std::vector<uint16_t>& vector)
        {
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for (auto iter = this->ray_trace->begin(); iter != this->ray_trace->end(); ++iter)
            {
                const bool on_positive_ray = iter->d > 0.0f;
                vector[iter->i] |=  (on_positive_ray * u16_viewed) +
                                   (!on_positive_ray * u16_occluded);
            }
        }


        void operator()(std::vector<uint32_t>& vector)
        {
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for (auto iter = this->ray_trace->begin(); iter != this->ray_trace->end(); ++iter)
            {
                const bool on_positive_ray = iter->d > 0.0f;
                vector[iter->i] |=  (on_positive_ray * u32_viewed) +
                                   (!on_positive_ray * u32_occluded);
            }
        }


        void operator()(std::vector<size_t>& vector)
        {
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for (auto iter = this->ray_trace->begin(); iter != this->ray_trace->end(); ++iter)
            {
                const bool on_positive_ray = iter->d > 0.0f;
                vector[iter->i] |=  (on_positive_ray * sz_viewed) +
                                   (!on_positive_ray * sz_occluded);
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(CountViews& caller)
            : caller(caller)
        {

        }


        /// @brief Reference to the specific derived class calling this object.
        CountViews& caller;
    };


    /// @brief This subclass provides update functions for post-update processing of the data.
    /// @note  This is still a subclass of `VoxelGrid::UpdateCallable` so unsupported vector types may use
    ///        the defaults error-throwing functions for unsupported types which the base class provides.
    struct PostUpdateCallable :  public VoxelGrid::UpdateCallable
    {
        using VoxelGrid::UpdateCallable::operator();

        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<uint8_t>& vector)
        {
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            this->caller.viewed_count   = 0;
            this->caller.occluded_count = 0;
            this->caller.unseen_count   = 0;
            for (auto& iter : vector)
            {
                const bool was_viewed   =  iter & u8_viewed;
                const bool was_occluded = (iter & u8_occluded) && !was_viewed;
                iter &= u8_ceiling;

                const bool no_overflow = iter != u8_ceiling;
                iter += (was_viewed & no_overflow) * 1;

                this->caller.viewed_count   += was_viewed;
                this->caller.occluded_count += was_occluded;
                this->caller.unseen_count   += !(was_viewed | was_occluded);
            }
        }


        void operator()(std::vector<uint16_t>& vector)
        {
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            this->caller.viewed_count   = 0;
            this->caller.occluded_count = 0;
            this->caller.unseen_count   = 0;
            for (auto& iter : vector)
            {
                const bool was_viewed   =  iter & u16_viewed;
                const bool was_occluded = (iter & u16_occluded) && !was_viewed;
                iter &= u16_ceiling;

                const bool no_overflow = iter != u16_ceiling;
                iter += (was_viewed & no_overflow) * 1;

                this->caller.viewed_count   += was_viewed;
                this->caller.occluded_count += was_occluded;
                this->caller.unseen_count   += !(was_viewed | was_occluded);
            }
        }


        void operator()(std::vector<uint32_t>& vector)
        {
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            this->caller.viewed_count   = 0;
            this->caller.occluded_count = 0;
            this->caller.unseen_count   = 0;
            for (auto& iter : vector)
            {
                const bool was_viewed   =  iter & u32_viewed;
                const bool was_occluded = (iter & u32_occluded) && !was_viewed;
                iter &= u32_ceiling;

                const bool no_overflow = iter != u32_ceiling;
                iter += (was_viewed & no_overflow) * 1;

                this->caller.viewed_count   += was_viewed;
                this->caller.occluded_count += was_occluded;
                this->caller.unseen_count   += !(was_viewed | was_occluded);
            }
        }


        void operator()(std::vector<size_t>& vector)
        {
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            this->caller.viewed_count   = 0;
            this->caller.occluded_count = 0;
            this->caller.unseen_count   = 0;
            for (auto& iter : vector)
            {
                const bool was_viewed   =  iter & sz_viewed;
                const bool was_occluded = (iter & sz_occluded) && !was_viewed;
                iter &= sz_ceiling;

                const bool no_overflow = iter != sz_ceiling;
                iter += (was_viewed & no_overflow) * 1;

                this->caller.viewed_count   += was_viewed;
                this->caller.occluded_count += was_occluded;
                this->caller.unseen_count   += !(was_viewed | was_occluded);
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an PostUpdateCallable to implement the derived class's post-update function.
        /// @param caller Reference to the specific derived class calling this object.
        PostUpdateCallable(CountViews& caller)
            : caller(caller)
        {

        }

        /// @brief Reference to the specific derived class calling this object.
        CountViews& caller;
    };

    static constexpr uint8_t u8_viewed   =           ~(std::numeric_limits<uint8_t>::max() >> 1);              // 1000'0000
    static constexpr uint8_t u8_occluded = (uint8_t)(~(std::numeric_limits<uint8_t>::max() >> 2) ^ u8_viewed); // 0100'0000
    static constexpr uint8_t u8_ceiling  =             std::numeric_limits<uint8_t>::max() >> 2;               // 0011'1111

    static constexpr uint16_t u16_viewed   =            ~(std::numeric_limits<uint16_t>::max() >> 1);
    static constexpr uint16_t u16_occluded = (uint16_t)(~(std::numeric_limits<uint16_t>::max() >> 2) ^ u16_viewed);
    static constexpr uint16_t u16_ceiling  =              std::numeric_limits<uint16_t>::max() >> 2;

    static constexpr uint32_t u32_viewed   =            ~(std::numeric_limits<uint32_t>::max() >> 1);
    static constexpr uint32_t u32_occluded = (uint32_t)(~(std::numeric_limits<uint32_t>::max() >> 2) ^ u32_viewed);
    static constexpr uint32_t u32_ceiling  =              std::numeric_limits<uint32_t>::max() >> 2;

    static constexpr size_t sz_viewed   =          ~(std::numeric_limits<size_t>::max() >> 1);
    static constexpr size_t sz_occluded = (size_t)(~(std::numeric_limits<size_t>::max() >> 2) ^ sz_viewed);
    static constexpr size_t sz_ceiling  =            std::numeric_limits<size_t>::max() >> 2;

    size_t occluded_count = 0, viewed_count = 0, unseen_count = 0;

    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This must be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;

    /// @brief Subclass callable that std::visit uses to modify the grid post-updates.
    PostUpdateCallable post_update_callable;
};


/// @brief String for the class name.
const std::string CountViews::type_name = "CountViews";


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_COUNT_VIEWS_HPP
