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
    /// @brief Constructor for a shared pointer to a Probability Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param parser Arg Parser with arguments to construct an Probability Grid from.
    /// @return Shared pointer to a Probability Grid.
    /// @throws `std::invalid_argument` if any probability values are not in the range 0<= p <= 1.
    /// @throws `std::invalid_argument` if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<Probability> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const utilities::ArgParser& parser)
    {
        return create(properties, parser.getCmdOption<float>("--dist-min", -0.2),
                                  parser.getCmdOption<float>("--dist-max",  0.2),
                                  parser.getCmdOption<float>("--p-max",    0.98f),
                                  parser.getCmdOption<float>("--p-min",    0.02f),
                                  parser.getCmdOption<float>("--p-past",   0.50f),
                                  parser.getCmdOption<float>("--p-sensed", 0.90f),
                                  parser.getCmdOption<float>("--p-far",    0.15f),
                                  parser.getCmdOption<float>("--p-init",   0.50f),
                                  stringToDataType(parser.getCmdOption("--data-type"), DataType::FLOAT));
    }


    /// @brief Constructor for a shared pointer to a Probability Voxel Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param dist_min   Minimum update distance. Default -0.2.
    /// @param dist_max   Maximum update distance. Default +0.2.
    /// @param p_max      Probability for voxel maximum voxel value saturation. Default 0.98.
    /// @param p_min      Probability for voxel minimum voxel value saturation. Default 0.02.
    /// @param p_past     Probability for voxels pst the sensed point. Default 0.50.
    /// @param p_sensed   Probability for the voxel at the sensed point. Default 0.90.
    /// @param p_far      Probability for voxels far in front of the sensed point. Default 0.15.
    /// @param p_init     Probability for voxel initialization. Default 0.50.
    /// @param type_id Datatype for the Grid. Default is float.
    /// @return Shared pointer to a Probability Grid.
    /// @throws `std::invalid_argument` if any probability values are not in the range 0<= p <= 1.
    /// @throws `std::invalid_argument` if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<Probability> create(const std::shared_ptr<const Grid::Properties>& properties,
                                               const float& dist_min = -0.2,
                                               const float& dist_max =  0.2,
                                               const float& p_max    =  0.98,
                                               const float& p_min    =  0.02,
                                               const float& p_past   =  0.50,
                                               const float& p_sensed =  0.90,
                                               const float& p_far    =  0.15,
                                               const float& p_init   =  0.50,
                                               const DataType& type_id = DataType::FLOAT)
    {
        return std::shared_ptr<Probability>(new Probability(properties, dist_min, dist_max,
                                                            std::clamp(p_max,    0.0f, 1.0f),
                                                            std::clamp(p_min,    0.0f, 1.0f),
                                                            std::clamp(p_past,   0.0f, 1.0f),
                                                            std::clamp(p_sensed, 0.0f, 1.0f),
                                                            std::clamp(p_far,    0.0f, 1.0f),
                                                            std::clamp(p_init,   0.0f, 1.0f),
                                                            type_id));
    }


    /// @brief Returns the class type name for the Voxel Grid.
    const std::string& getTypeName() const override final
    {
        static const std::string name = "Probability";
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
    /// @param p_max      Probability for voxel maximum voxel value saturation.
    /// @param p_min      Probability for voxel minimum voxel value saturation.
    /// @param p_past     Probability for voxels pst the sensed point.
    /// @param p_sensed   Probability for the voxel at the sensed point.
    /// @param p_far      Probability for voxels far in front of the sensed point.
    /// @param p_init     Probability for voxel initialization.
    /// @param type_id Datatype for the Grid.
    /// @throws `std::invalid_argument` if any probability values are not in the range 0<= p <= 1.
    /// @throws `std::invalid_argument` if the DataType is not supported by this VoxelGrid.
    explicit Probability(const std::shared_ptr<const Grid::Properties>& properties,
                         const float& dist_min,
                         const float& dist_max,
                         const float& p_max,
                         const float& p_min,
                         const float& p_past,
                         const float& p_sensed,
                         const float& p_far,
                         const float& p_init,
                         const DataType& type_id)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    p_init,
                    type_id,
                    DataType::TYPE_FLOATING_POINT),
          p_max(p_max),
          p_min(p_min),
          p_past(p_past),
          p_sensed(p_sensed),
          p_far(p_far),
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


        void operator()(std::vector<float>& vector)
        {
            using namespace forge_scan::utilities::math;

            trace::const_iterator iter = ray_trace::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end())
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                float px = this->get_px(iter);
                float px = this->caller.p_far;
                if(iter->second <= 0)
                {
                    float dx = std::abs(iter->second / this->caller.dist_min);
                    px = lerp(this->caller.p_sensed, this->caller.p_past, dx);
                }
                else if (iter->second <= this->caller.dist_max)
                {
                    float dx = std::abs(iter->second / this->caller.dist_max);
                    px = lerp(this->caller.p_sensed, this->caller.p_far, dx);
                }
                vector[iter->first] = std::clamp(probability(log_odds(vector[iter->first]) + log_odds(px)),
                                                 this->caller.p_min, this->caller.p_max);
            }
        }


        void operator()(std::vector<double>& vector)
        {
            using namespace forge_scan::utilities::math;

            trace::const_iterator iter = ray_trace::first_above_min_dist(this->ray_trace, this->caller.dist_min);
            for (; ; ++iter)
            {
                if (iter == this->ray_trace->end())
                {
                    return;
                }
                // ************************** APPLY VOXEL UPDATE HERE ************************** //
                float px = this->get_px(iter);
            }
        }


        /// @brief Gets the occupation probability for a location on the ray.
        /// @param iter Iterator for the ray trace.
        /// @return Occupation probability for the iterator's location on the ray.  
        float get_px(const trace::const_iterator& iter)
        {
            using namespace forge_scan::utilities::math;

                if(iter->second <= 0)
                {
                    float dx = std::abs(iter->second / this->caller.dist_min);
                return lerp(this->caller.p_sensed, this->caller.p_past, dx);
                }
                else if (iter->second <= this->caller.dist_max)
                {
                    float dx = std::abs(iter->second / this->caller.dist_max);
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


private:
    /// @brief Probability for maximum voxel value saturation.
    const float p_max;

    /// @brief Probability for minimum voxel values saturation.
    const float p_min;

    /// @brief Probability for voxels past the sensed point
    const float p_past;

    /// @brief Probability for sensed point.
    const float p_sensed;

    /// @brief Probability for voxels far in front of the sensed point. 
    const float p_far;

    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This musts be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};



} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_PROBABILITY_HPP
