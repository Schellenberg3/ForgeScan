#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_PROBABILITY_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_PROBABILITY_HPP

#include <algorithm>

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"
#include "ForgeScan/Utilities/Math.hpp"


namespace forge_scan {
namespace data {


/// @brief Represents occupation probability via log-odds.
///        This implements similar logic to what the OctoMap library uses.
/// @note Supports `float` and `double` data types only.
class Probability : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to a Probability VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param parser ArgParser with arguments to construct an Probability Grid from.
    /// @return Shared pointer to a Probability Grid.
    /// @throws std::invalid_argument if any probability values are not in the range 0<= p <= 1.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<Probability> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const utilities::ArgParser& parser)
    {
        return create(properties, parser.get<float>(VoxelGrid::parse_d_min, VoxelGrid::default_d_min),
                                  parser.get<float>(VoxelGrid::parse_d_max, VoxelGrid::default_d_max),
                                  parser.get<float>(Probability::parse_p_max,    Probability::default_p_max),
                                  parser.get<float>(Probability::parse_p_min,    Probability::default_p_min),
                                  parser.get<float>(Probability::parse_p_past,   Probability::default_p_past),
                                  parser.get<float>(Probability::parse_p_sensed, Probability::default_p_sensed),
                                  parser.get<float>(Probability::parse_p_far,    Probability::default_p_far),
                                  parser.get<float>(Probability::parse_p_init,   Probability::default_p_init),
                                  parser.get<float>(Probability::parse_p_thresh, Probability::default_p_thresh),
                                  parser.has(Probability::parse_save_as_log_odds),
                                  stringToDataType(parser.get(VoxelGrid::parse_dtype), DataType::FLOAT));
    }


    /// @brief Constructor for a shared pointer to a Probability VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min   Minimum update distance. Default -0.2.
    /// @param dist_max   Maximum update distance. Default +0.2.
    /// @param p_max      Probability for voxel maximum voxel value saturation. Default 0.98.
    /// @param p_min      Probability for voxel minimum voxel value saturation. Default 0.02.
    /// @param p_past     Probability for voxels pst the sensed point. Default 0.50.
    /// @param p_sensed   Probability for the voxel at the sensed point. Default 0.90.
    /// @param p_far      Probability for voxels far in front of the sensed point. Default 0.15.
    /// @param p_init     Probability for voxel initialization. Default 0.50.
    /// @param save_as_log_odds If true then the intermediate log odds value is stored. Other wise the
    ///                        voxels are converted to probability, saved, then back to log odds.
    /// @param type_id Datatype for the Grid. Default is float.
    /// @return Shared pointer to a Probability Grid.
    /// @note Probability values clamped to the range 0<= p <= 1.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<Probability> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const float& dist_min = -0.2,
                                               const float& dist_max =  0.2,
                                               const float& p_max    =  Probability::default_p_max,
                                               const float& p_min    =  Probability::default_p_min,
                                               const float& p_past   =  Probability::default_p_past,
                                               const float& p_sensed =  Probability::default_p_sensed,
                                               const float& p_far    =  Probability::default_p_far,
                                               const float& p_init   =  Probability::default_p_init,
                                               const float& p_thresh =  Probability::default_p_thresh,
                                               const bool& save_as_log_odds = false,
                                               const DataType& type_id = DataType::FLOAT)
    {
        return std::shared_ptr<Probability>(new Probability(properties, dist_min, dist_max,
                                                            std::clamp(p_max,    0.0f, 1.0f),
                                                            std::clamp(p_min,    0.0f, 1.0f),
                                                            std::clamp(p_past,   0.0f, 1.0f),
                                                            std::clamp(p_sensed, 0.0f, 1.0f),
                                                            std::clamp(p_far,    0.0f, 1.0f),
                                                            std::clamp(p_init,   0.0f, 1.0f),
                                                            std::clamp(p_thresh, 0.0f, 1.0f),
                                                            save_as_log_odds,
                                                            type_id));
    }


    /// @return Help message for constructing a Probability VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: Probability help message";
    }


    /// @brief Returns the class type name for the VoxelGrid.
    const std::string& getTypeName() const override final
    {
        return Probability::type_name;
    }

    /// @brief Accessor for `metrics::ground_truth::ExperimentOccupancy` in
    ///       `metrics::OccupancyConfusion`
    /// @return Occupancy data vector.
    std::vector<uint8_t> getOccupancyData() const
    {
        auto occupancy_data = std::vector<uint8_t>(this->properties->getNumVoxels(), VoxelOccupancy::UNSEEN);

        auto get_occupancy_data = [&](auto&& data){
            for (size_t i = 0; i < data.size(); ++i)
            {
                if (data[i] < this->log_p_thresh)
                {
                    occupancy_data[i] = VoxelOccupancy::FREE;
                }
            }
        };

        auto get_occupancy_data_with_seen_info = [&](auto&& data){
            for (size_t i = 0; i < data.size(); ++i)
            {
                if (data[i] < this->log_p_thresh) // || (data[i] == this->log_p_init && this->data_seen->operator[](i) == true))
                {
                    occupancy_data[i] = VoxelOccupancy::FREE;
                }
                else // if (data[i] != this->log_p_init)
                {
                    occupancy_data[i] = VoxelOccupancy::OCCUPIED;
                }
            }
        };

        if (this->data_seen != nullptr && this->data_seen->size() == this->properties->getNumVoxels())
        {
            std::visit(get_occupancy_data_with_seen_info, this->data);
        }
        else
        {
            std::visit(get_occupancy_data, this->data);
        }
        return occupancy_data;
    }


    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(const std::shared_ptr<const Trace>& ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }


    static const float default_p_max, default_p_min,  default_p_past, default_p_sensed,
                       default_p_far, default_p_init, default_p_thresh;

    static const std::string parse_p_max, parse_p_min,  parse_p_past, parse_p_sensed,
                             parse_p_far, parse_p_init, parse_p_thresh, parse_save_as_log_odds;

    static const std::string type_name;

private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min   Minimum trace update distance for this VoxelGrid.
    /// @param dist_max   Maximum trace update distance for this VoxelGrid.
    /// @param p_max      Probability for voxel maximum voxel value saturation.
    /// @param p_min      Probability for voxel minimum voxel value saturation.
    /// @param p_past     Probability for voxels pst the sensed point.
    /// @param p_sensed   Probability for the voxel at the sensed point.
    /// @param p_far      Probability for voxels far in front of the sensed point.
    /// @param p_init     Probability for voxel initialization.
    /// @param save_as_log_odd If true then the intermediate log odds value is stored. Other wise the
    ///                        voxels are converted to probability, saved, then back to log odds.
    /// @param type_id Datatype for the Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    explicit Probability(const std::shared_ptr<const Grid::Properties>& properties,
                         const float& dist_min,
                         const float& dist_max,
                         const float& p_max,
                         const float& p_min,
                         const float& p_past,
                         const float& p_sensed,
                         const float& p_far,
                         const float& p_init,
                         const float& p_thresh,
                         const bool& save_as_log_odds,
                         const DataType& type_id)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    utilities::math::log_odds(p_init),
                    type_id,
                    DataType::TYPE_FLOATING_POINT),
          log_p_max(utilities::math::log_odds(p_max)),
          log_p_min(utilities::math::log_odds(p_min)),
          log_p_init(utilities::math::log_odds(p_init)),
          p_past(p_past),
          p_sensed(p_sensed),
          p_far(p_far),
          log_p_thresh(utilities::math::log_odds(p_thresh)),
          save_as_log_odds(save_as_log_odds),
          update_callable(*this),
          update_callable_converter(*this)
    {

    }


    /// @note This is virtual so VoxelGrid with multiple data channels may specifically handle
    ///       their channels. But most derived VoxelGrids may uses this method.
    void save(HighFive::Group& g_channel, const std::string& grid_type) override final
    {
        if (this->save_as_log_odds == false)
        {
            this->update_callable_converter.setToProbability();
            std::visit(this->update_callable_converter, this->data);
        }
        VoxelGrid::save(g_channel, grid_type);
        if (this->save_as_log_odds == false)
        {
            this->update_callable_converter.setToLogOdds();
            std::visit(this->update_callable_converter, this->data);
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
            using namespace forge_scan::utilities::math;

            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->end();

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                float px = this->get_px(iter);
                vector[iter->i] = std::clamp(vector[iter->i] + log_odds(px),
                                             this->caller.log_p_min, this->caller.log_p_max);
            }
        }


        void operator()(std::vector<double>& vector)
        {
            using namespace forge_scan::utilities::math;

            Trace::const_iterator       iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last = this->ray_trace->end();

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last; ++iter)
            {
                float px = this->get_px(iter);
                vector[iter->i] = std::clamp(vector[iter->i] + log_odds(px),
                                             static_cast<double>(this->caller.log_p_min),
                                             static_cast<double>(this->caller.log_p_max));
            }
        }


        /// @brief Gets the occupation probability for a location on the ray.
        /// @param iter Iterator for the ray trace.
        /// @return Occupation probability for the iterator's location on the ray.
        float get_px(const Trace::const_iterator& iter)
        {
            using namespace forge_scan::utilities::math;

            if(iter->d <= 0)
            {
                float dx = std::abs(iter->d / this->caller.dist_min);
                return lerp(this->caller.p_sensed, this->caller.p_past, dx);
            }
            else if (iter->d <= this->caller.dist_max)
            {
                float dx = std::abs(iter->d / this->caller.dist_max);
                return lerp(this->caller.p_sensed, this->caller.p_far, dx);
            }
            return this->caller.p_far;
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(Probability& caller)
            : caller(caller)
        {

        }

        /// @brief Reference to the specific derived class calling this object.
        Probability& caller;
    };


    /// @brief Subclass provides update functions for each supported DataType/VectorVariant of
    ///        the data vector.
    struct UpdateCallableConverter : public VoxelGrid::UpdateCallable
    {
        using VoxelGrid::UpdateCallable::operator();


        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<float>& vector)
        {
            using namespace forge_scan::utilities::math;
            using namespace std::placeholders;

            auto convert = std::bind(this->to_probability ? probability<float> : log_odds<float>, _1);
            for (auto& item : vector)
            {
                item = convert(item);
            }
        }


        void operator()(std::vector<double>& vector)
        {
            using namespace forge_scan::utilities::math;
            using namespace std::placeholders;

            auto convert = std::bind(this->to_probability ? probability<double> : log_odds<double>, _1);
            for (auto& item : vector)
            {
                item = convert(item);
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallableConverter(Probability& caller)
            : caller(caller)
        {

        }

        void setToProbability() { this->to_probability = true; }
        
        void setToLogOdds() { this->to_probability = false; }

        /// @brief Reference to the specific derived class calling this object.
        Probability& caller;

        bool to_probability = true;
    };


private:
    /// @brief Log probability for maximum voxel value saturation.
    const float log_p_max;

    /// @brief Log probability for minimum voxel values saturation.
    const float log_p_min;

    /// @brief Log probability for initial voxel values.
    const float log_p_init;

    /// @brief Probability for voxels past the sensed point
    const float p_past;

    /// @brief Probability for sensed point.
    const float p_sensed;

    /// @brief Probability for voxels far in front of the sensed point.
    const float p_far;

    /// @brief Log probability for threshold above which voxels are consider occupied.
    const float log_p_thresh;

    /// @brief Controls how the data is saved. If true log odds are saved.
    const bool save_as_log_odds;

    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This musts be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;

    UpdateCallableConverter update_callable_converter;
};


/// @brief String for the class name.
const std::string Probability::type_name = "Probability";

/// @brief Default occupation probability threshold values.
const float Probability::default_p_max    = 0.98f,
            Probability::default_p_min    = 0.02f;

/// @brief Default occupation probability values at specific positions along a ray.
const float Probability::default_p_past   = 0.60f,
            Probability::default_p_sensed = 0.80f,
            Probability::default_p_far    = 0.10f,
            Probability::default_p_init   = 0.60f,
            Probability::default_p_thresh = 0.51f;

/// @brief ArgParser key for the maximum occupation probability a voxel may have.
const std::string Probability::parse_p_max = "--p-max";

/// @brief ArgParser key for the minimum occupation probability a voxel may have.
const std::string Probability::parse_p_min = "--p-min";

/// @brief ArgParser key for the occupation probability at d-min.
const std::string Probability::parse_p_past = "--p-past";

/// @brief ArgParser key for the occupation probability at the sensed point.
const std::string Probability::parse_p_sensed = "--p-sensed";

/// @brief ArgParser key for the occupation probability at d-max and above.
const std::string Probability::parse_p_far = "--p-far";

/// @brief ArgParser key for the occupation probability to initialize a voxel to.
const std::string Probability::parse_p_init = "--p-init";

/// @brief ArgParser key for the occupation probability above which voxels are consider occupied.
const std::string Probability::parse_p_thresh = "--p-thresh";

/// @brief ArgParser flag for saving the voxels as the intermediate log odds instead of converting
///        to (and then back from) the probability.
const std::string Probability::parse_save_as_log_odds = "--save-as-log-odds";


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_PROBABILITY_HPP
