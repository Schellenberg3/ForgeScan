#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_TSDF_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_TSDF_HPP

#include <functional>

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
    /// @brief Constructor for a shared pointer to a TSDF VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param parser ArgParser with arguments to construct an TSDF Grid from.
    /// @return Shared pointer to a TSDF Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                        const utilities::ArgParser& parser)
    {
        return create(properties, parser.get<float>(VoxelGrid::parse_d_min, VoxelGrid::default_d_min),
                                  parser.get<float>(VoxelGrid::parse_d_max, VoxelGrid::default_d_max),
                                  parser.has(TSDF::parse_average),
                                  stringToDataType(parser.get(VoxelGrid::parse_dtype), DataType::FLOAT));
    }


    /// @brief Constructor for a shared pointer to a TSDF VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min   Minimum update distance. Default -0.2.
    /// @param dist_max   Maximum update distance. Default +0.2.
    /// @param average  Records average TSDF value if true. Otherwise records minimum magnitude. Default true.
    /// @param type_id Datatype for the Grid. Default is float.
    /// @return Shared pointer to a TSDF Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                        const float& dist_min = -0.2,
                                        const float& dist_max =  0.2,
                                        const bool& average   = true,
                                        const DataType& type_id = DataType::FLOAT)
    {
        float default_value = average ? 0.0f : NEGATIVE_INFINITY;
        return std::shared_ptr<TSDF>(new TSDF(properties, dist_min, dist_max, average, default_value, type_id));
    }


    /// @return Help message for constructing a TSDF VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: TSDF help message";
    }


    /// @brief Returns the class type name for the VoxelGrid.
    const std::string& getTypeName() const override final
    {
        return TSDF::type_name;
    }


    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(const std::shared_ptr<const Trace>& ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }

    static const std::string parse_average;

    static const std::string type_name;

private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min Minimum trace update distance for this VoxelGrid.
    /// @param dist_max Maximum trace update distance for this VoxelGrid.
    /// @param average  Records average TSDF value if true. Otherwise records minimum magnitude.
    /// @param default_value  Initialization value for all voxels in this VoxelGrid. Should be 0 for
    ///                       average or -INF for minimum magnitude.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    explicit TSDF(const std::shared_ptr<const Grid::Properties>& properties,
                  const float& dist_min,
                  const float& dist_max,
                  const bool&  average,
                  const float& default_value,
                  const DataType& type_id)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    default_value,
                    type_id,
                    DataType::TYPE_FLOATING_POINT),
          average(average),
          update_callable(*this)
    {
        if (this->average)
        {
            this->sample_count = std::vector<size_t>(this->properties->getNumVoxels(), 0);
            this->variance     = std::vector<float>(this->properties->getNumVoxels(), 0.0f);
        }
    }


    /// @brief Subclass provides update functions for each supported DataType/VectorVariant of
    ///        the data vector.
    struct UpdateCallable : public VoxelGrid::UpdateCallable
    {
        using VoxelGrid::UpdateCallable::operator();

        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<float>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                this->update_callback(vector[iter->i], iter->d, iter->i);
            }
        }


        void operator()(std::vector<double>& vector)
        {
            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->first_above(this->caller.dist_max, iter);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                float average = static_cast<float>(vector[iter->i]);
                this->update_callback(average, iter->d, iter->i);
                vector[iter->i] = static_cast<double>(average);
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(TSDF& caller)
            : caller(caller)
        {
            if (this->caller.average)
            {
                assert(this->caller.sample_count.size() == this->caller.properties->getNumVoxel());
                assert(this->caller.variance.size() == this->caller.properties->getNumVoxels());
                this->update_callback = std::bind(&UpdateCallable::update_average, this,
                                                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            }
            else
            {
                this->update_callback = std::bind(&UpdateCallable::update_min_magnitude, this,
                                                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            }
        }


        /// @brief Performs TSDF voxel value update for minimum distance.
        /// @param [out] original Current voxel distance value. Updated in place.
        /// @param update Newly measured TSDF distance.
        /// @param i Unused. Vector index for the voxel. See `Grid::Properties::at`.
        void update_min_magnitude(float& original, const float& update, [[maybe_unused]] const size_t& i)
        {
            original = utilities::math::smallest_magnitude(original, update);
        }


        /// @brief Performs TSDF voxel value update for average distance.
        /// @param [out] average Current average value. Updated in place.
        /// @param update Newly measured TSDF distance.
        /// @param i Vector index for the voxel. See `Grid::Properties::at`.
        /// @details Uses a (modified) version of Welford's algorithm for online updates of the average and variance. See:
        ///          https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
        void update_average(float& average, const float& update, const size_t& i)
        {
            float&  var = this->caller.variance[i];
            size_t& n   = this->caller.sample_count[i];

            float delta = update - average;
            
            var     *= n;
            average += delta / ++n;
            var     += delta * delta;
            var     /= n;
        }


        /// @brief Reference to the specific derived class calling this object.
        TSDF& caller;

        /// @brief Callback for the update method: `update_average` or `update_min_magnitude`.
        std::function<void(float&, const float&, const size_t&)> update_callback;
    };


    /// @brief If true, uses the average TSDF update algorithm rather than minimum magnitude.
    const bool average;

    /// @brief Stores the update count data that the grid uses.
    std::vector<size_t> sample_count;

    /// @brief Stores the update count data that the grid uses.
    std::vector<float> variance;

    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This musts be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};


/// @brief String for the class name.
const std::string TSDF::type_name = "TSDF";

/// @brief ArgParser flag for using an average method rather than a minimum magnitude method.
const std::string TSDF::parse_average = "--average";


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_UPDATE_COUNT_HPP
