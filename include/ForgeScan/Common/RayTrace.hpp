#ifndef FORGE_SCAN_COMMON_RAY_TRACE_HPP
#define FORGE_SCAN_COMMON_RAY_TRACE_HPP

#include <cstddef>

#include "ForgeScan/Common/AABB.hpp"
#include "ForgeScan/Common/Grid.hpp"
#include "ForgeScan/Common/VectorMath.hpp"


// *********************************************************************************************************************** //
// * This implements the ray traversal of a VoxelGrid, as used by the Reconstruction class.                             * //
// *                                                                                                                     * //
// * The methods in this file follow the the Amanatides-Woo algorithm for fast voxel traversal. See:                     * //
// *     http://www.cse.yorku.ca/~amana/research/grid.pdf                                                                * //
// *                                                                                                                     * //
// * Essentially, this treats the input ray as a parametrized line between the start and end point. The algorithm must   * //
// * decide what direction - X, Y, or Z - to step in. For each direction, the track a parameter, `dist`, for the         * //
// * distance traveled from the starting voxel in that direction. The direction that has the smallest `dist` value is    * //
// * selected to `step` in. This either increments or decrements the index in that direction.                            * //
// *                                                                                                                     * //
// * When a step is take the `dist` value for that direction is updated. This update, `delta`, is equal to the           * //
// * voxel resolution in that direction divided by the normal of the ray. This describes how many units of the vector    * //
// * must be walked to traverse one voxel unit.                                                                          * //
// *                                                                                                                     * //
// * For example if the `normal` in a direction is large (meaning the vector nearly points along the direction), the     * //
// * `delta` will be small and the algorithm will take many steps in that direction before choosing another one.         * //
// *                                                                                                                     * //
// * It is critical for this method that the grid of voxels may is considered as an axis-aligned bounding-box (AABB).    * //
// * This means any input data must be properly transformed before reaching this algorithm.                              * //
// *                                                                                                                     * //
// * The algorithm also considers that the voxel "origin" is the lower-bounding corner of the voxel. If the rest of the  * //
// * code considers the "origin" to be the center of the voxel then a reference frame shift of half the voxel resolution * //
// * must be applied to the ray's start point when calculating `dist`. See `correct_traversal_parameters` for more.      * //
// *********************************************************************************************************************** //


namespace forge_scan {


/// @brief Voxel information within a Trace.
struct TraceVoxel
{
    TraceVoxel(const size_t& i, const float& d)
        : i(i), d(d)
    {

    }

    /// @brief Vector index for the voxel. See `Grid::Properties::at`.
    size_t i;

    /// @brief Distance from the voxel to the sensed point.
    float  d;
};


/// @brief Collection of voxels hit by a ray. Stores pairs of the voxel's vector index and its
///        distance from the ray's sended point.
/// @note  This is sorted in ascending distance from the sensed point.
struct Trace : public std::vector<TraceVoxel>
{
    /// @details Required to set the sensed iterator for the Trace.
    friend bool get_ray_trace(const std::shared_ptr<Trace>&, const Point&, const Point&,
                              const std::shared_ptr<const Grid::Properties>&, const float&, const float&);


    /// @brief Describes where the sensed point is relative to the traced ray.
    enum SensedLocation : uint8_t
    {
        UNKNOWN = 0b0000'0011,
        BEFORE  = 0b0000'0001,
        AFTER   = 0b0000'0010,
        IN      = 0b0000'0000
    };


    /// @brief Empties the contents of the vector and resets information about the sensed point.
    void clear()
    {
        std::vector<TraceVoxel>::clear();
        this->sensed_location = SensedLocation::UNKNOWN;
        this->sensed_point    = Point(-1, -1, -1);
    }


    /// @brief Checks if the sensed point is on the trace.
    /// @return True if the sensed point is on the trace. False if the sensed point is before or
    ///         after the trace.
    bool hasSensed() const
    {
        return this->sensed_location == SensedLocation::IN;
    }


    /// @brief  Gets the point location for the ray's sensed point. The distance here is implicitly zero.
    /// @return Read only reference to the ray's sensed point location.
    const Point& sensedPoint() const
    {
       return this->sensed_point;
    }


    /// @brief Finds the voxel at a distance greater than the specified value.
    /// @param dist Threshold distance.
    /// @return First voxel with a value greater than the specified distance threshold.
    ///         Or the `end` iterator if all values are below the distance threshold.
    Trace::const_iterator first_above(const float& dist) const
    {
        Trace::const_iterator iter = this->begin();
        this->increment_to_first_above(iter, dist);
        return iter;
    }


    /// @brief Finds the voxel at a distance greater than the specified value.
    /// @param dist  Threshold distance.
    /// @param start Iterator to begin the search from.
    /// @return First voxel with a value greater than the specified distance threshold.
    ///         Or the `end` iterator if all values are below the distance threshold.
    /// @warning If the `start` iterator does not point to this Trace or this Trace has moved
    ///          since the iterator's creation then this will have unintended consequences.
    ///          Likely an infinite loop but return values should be treated as undefined behavior.
    Trace::const_iterator first_above(const float& dist, const Trace::const_iterator& start) const
    {
        Trace::const_iterator iter = start;
        this->increment_to_first_above(iter, dist);
        return iter;
    }


private:
    /// @brief Implements finding the voxel at a distance greater than the specified value.
    /// @param iter Vector iterator (constant).
    /// @param dist Threshold distance.
    /// @return First voxel with a value greater than the specified distance threshold.
    ///         Or the `end` iterator if all values are below the distance threshold.
    void increment_to_first_above(Trace::const_iterator& iter, const float& dist) const
    {
        if (dist == INFINITY)
        {
            iter = this->end();
        }
        else
        {
            while (iter != this->end() && iter->d < dist)
            {
                ++iter;
            }
        }
    }


    /// @brief Sets the sensed_iter used by the `sensed` function.
    /// @param sensed_point    The location which this ray measured, relative the the grid.
    /// @param sensed_location Flag describing the sensed point's position relative to the traced ray.
    void set_sensed(const Point& sensed_point, const SensedLocation& sensed_location)
    {
        if (sensed_location == SensedLocation::UNKNOWN)
        {
            throw std::runtime_error("Cannot set with an unknown sensed location.");
        }
        this->sensed_location = sensed_location;
        this->sensed_point    = sensed_point;
    }

    /// @brief The measured surface point for the ray.
    Point sensed_point = Point(-1, -1, -1);


    /// @brief Flag for the status of the sensed point relative to the Trace.
    SensedLocation sensed_location = SensedLocation::UNKNOWN;
};


namespace ray_trace_helpers {


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline int get_step(const std::ptrdiff_t& d, const std::ptrdiff_t* sign)
{
    static constexpr int STEP_DIR[2] = {1, -1};
    return STEP_DIR[sign[d]];
}


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline float get_delta(const std::ptrdiff_t& d, const Direction& inv_normal,
                       const std::shared_ptr<const Grid::Properties>& properties)
{
    return std::abs(properties->resolution * inv_normal[d]);
}


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline float get_dist(const std::ptrdiff_t& d, const std::ptrdiff_t* sign, const Index& c_idx,
                      const Point& sensed_adj, const Direction& inv_normal, const float& dist_min_adj,
                      const std::shared_ptr<const Grid::Properties>& properties)
{
    static constexpr float NEXT_ADJ[2] = {0.5, -0.5};
    return dist_min_adj + ((c_idx[d] + NEXT_ADJ[sign[d]]) * properties->resolution - sensed_adj[d]) * inv_normal[d];
}


/// @brief Helper for `get_ray_trace`.
/// @warning This should only be called by `get_ray_trace`.
inline std::ptrdiff_t get_min_dist(const float* dist)
{
    static constexpr std::ptrdiff_t X = 0, Y = 1, Z = 2;
    return (dist[X] < dist[Y] && dist[X] < dist[Z]) ? X : (dist[Y] < dist[Z]) ? Y : Z;
}


/// @brief Gets the correct enumeration value for the location of the sensed voxel, relative to the
///        rest of the Trace.
/// @param d_min Distance parameter for the minimum of the trace.
/// @param d_max Distance parameter for the maximum of the trace.
/// @return One of SensedLocation IN, BEFORE, or AFTER as appropriate.
/// @note The distance parameter for the sensed location is implicitly at `0`.
/// @warning This should only be called by `get_ray_trace`.
inline Trace::SensedLocation get_sensed_location(const float& d_min,  const float& d_max)
{
    uint8_t l = Trace::SensedLocation::UNKNOWN;
    l ^= Trace::SensedLocation::BEFORE*(d_min <= 0.0);   // Rules out the sensed point being before the trace.
    l ^= Trace::SensedLocation::AFTER *(0.0   <= d_max); // Rules out the sensed point being after the trace.
    assert(l != Trace::SensedLocation::UNKNOWN &&
           "Sensed location cannot be both after and before the trace.");
    return static_cast<Trace::SensedLocation>(l);
}


} // namespace ray_trace_helpers


/// @brief Calculates what voxels are hit on the ray between `sensed` and `origin`.
/// @param [out] ray_trace A trace of what voxels were hit and the distance from that voxel to the `sensed` voxel.
/// @param sensed Sensed point, the start of the ray.
/// @param origin Origin point, the end of the ray.
/// @param properties Shared `Grid::Properties` for the VoxelGrids begin traversed.
/// @param dist_min Minimum distance to trace along the ray, relative to the `sensed` point.
/// @param dist_max Maximum distance to trace along the ray, relative to the `sensed` point.
/// @return True if the ray intersected the Grid, this indicates that `ray_trace` has valid data to add.
inline bool get_ray_trace(const std::shared_ptr<Trace>& ray_trace,
                          const Point& sensed, const Point& origin,
                          const std::shared_ptr<const Grid::Properties>& properties,
                          const float& dist_min, const float& dist_max)
{
    static constexpr std::ptrdiff_t X = 0, Y = 1, Z = 2;

    ray_trace->clear();

    // Adjusted min and max distances so we only trace the ray while it is within the Grid's bounds.
    float dist_min_adj, dist_max_adj;

    float length;
    Direction normal, inv_normal;
    vector_math::get_length_normal_and_inverse_normal(sensed, origin, length, normal, inv_normal);
    length = std::min(length, dist_max);

    const bool valid_intersection = AABB::find_zero_bounded_intersection(properties->dimensions, sensed, inv_normal,
                                                                         dist_min, length, dist_min_adj, dist_max_adj);
    if (valid_intersection)
    {
        dist_min_adj = std::max(dist_min_adj, dist_min);
        dist_max_adj = std::min(dist_max_adj, dist_max);

        const Point sensed_adj = sensed + normal * dist_min_adj;
        Index c_idx = properties->pointToIndex(sensed_adj);

        const std::ptrdiff_t sign[3] = {std::signbit(normal[X]), std::signbit(normal[Y]), std::signbit(normal[Z])};

        // Direction of travel (increment or decrement) along the respective axis.
        const int step[3] = { ray_trace_helpers::get_step(X, sign),
                              ray_trace_helpers::get_step(Y, sign),
                              ray_trace_helpers::get_step(Z, sign) };

        // The amount of distance to move one voxel length along each axis based on the ray's direction.
        const float delta[3] = { ray_trace_helpers::get_delta(X, inv_normal, properties),
                                 ray_trace_helpers::get_delta(Y, inv_normal, properties),
                                 ray_trace_helpers::get_delta(Z, inv_normal, properties) };

        // Cumulative distance traveled along the respective axis.
        float dist[3] = { ray_trace_helpers::get_dist(X, sign, c_idx, sensed_adj, inv_normal, dist_min_adj, properties),
                          ray_trace_helpers::get_dist(Y, sign, c_idx, sensed_adj, inv_normal, dist_min_adj, properties),
                          ray_trace_helpers::get_dist(Z, sign, c_idx, sensed_adj, inv_normal, dist_min_adj, properties) };

        try
        {
            ray_trace->emplace_back(properties->at(c_idx), dist_min_adj);

            std::ptrdiff_t i = ray_trace_helpers::get_min_dist(dist);
            while (dist[i] <= dist_max_adj)
            {
                c_idx[i] +=  step[i];
                ray_trace->emplace_back(properties->at(c_idx), dist[i]);

                dist[i]  += delta[i];
                i = ray_trace_helpers::get_min_dist(dist);
            }
        }
        catch (const VoxelOutOfRange& e)
        {
            /// TODO: Synthetic data from the Camera class with high levels of noise (0.2 to 0.5) can cause this
            ///       loop to access (-1, -1, -1) and fail (will appear as 9223372036854775808 due to unsigned ints).
            ///       For now solving this is not important but it is curious.
            // Algorithm should never go out of bounds. But catching here dose not impact performance and prevents
            // undefined behavior and silent errors in a VoxelGrid update where all indicies are assumed to be valid.
            throw VoxelOutOfRange("Ray tracing failed: This should not happen. Failed with: " +  std::string(e.what()));
        }

        ray_trace->set_sensed(sensed, ray_trace_helpers::get_sensed_location(dist_min_adj, dist_max_adj));
    }
    return valid_intersection;
}


} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_RAY_TRACE_HPP
