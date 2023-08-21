#ifndef FORGE_SCAN_RECONSTRUCTION_RAY_TRACING_GRID_TRAVERSAL_HPP
#define FORGE_SCAN_RECONSTRUCTION_RAY_TRACING_GRID_TRAVERSAL_HPP

#include <cstddef>

#include "ForgeScan/Common/AABB.hpp"
#include "ForgeScan/Common/Grid.hpp"
#include "ForgeScan/Common/VectorMath.hpp"


namespace forge_scan {
namespace data {
namespace ray_trace {


// *********************************************************************************************************************** //
// * This implements the ray traversal of a Voxel Grid, as used by the Reconstruction class.                             * //
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


/// Index constants.
static constexpr std::ptrdiff_t X = 0, Y = 1, Z = 2;


/// @brief Finds the first item in the trace with a distance greater than the specified value.
/// @param ray_trace A shared, constant trace to search through.
/// @param min_dist  Lower bound distance to find.
/// @return Constant iterator for `ray_trace` which points either to the first element greater than
///         `min_dist` or to the end of ray_trace.
inline trace::const_iterator first_above_min_dist(std::shared_ptr<const trace> ray_trace, const float& min_dist)
{
    trace::const_iterator iter = ray_trace->begin();
    while (iter != ray_trace->end())
    {
        if (iter->second >= min_dist)
        {
            return iter;
        }
        ++iter;
    }
    return iter;
}


/// @brief Takes voxel traversal parameters and updates them in case initial assumptions were off.
/// @param step[out]   Direction of travel along each axis.
//                     Must decrement if the normal was negative.
/// @param delta[out]  Value to update `dist` by when a step is taken.
///                    Must ensure this is positive if the normal was negative.
/// @param dist[out]   Initial distance from the starting Point to the next voxel in each direction.
/// @param resolution  The resolution of the Grid - edge length of the square voxels.
/// @param start_voxel The voxel the vector starts in.
/// @param start_point Starting position of the vector.
/// @param normal      Normal vector for the ray.
/// @param inv_normal  Pre-computed inverse of the normal vector.
/// @warning This function is a helper for `get_ray_trace` and should not be called on its own.
inline void correct_traversal_parameters(int step[3], float delta[3], float dist[3], const float& resolution,
                                         const Index& start_voxel, const Point& start_point,
                                         const Direction& normal, const Direction& inv_normal)
{
    // The voxel traversal algorithm assumes a voxel's origin to be at the lower bound, rather than its center.
    // To calculate the traversal information we must virtually shift the initial Point.
    const Point start_point_shift = start_point.array() + 0.5 * resolution;

    if (normal[X] > 0)
    {
        // Update `dis`t with how far it is to the beginning voxel boundary.
        dist[X]  += ((start_voxel[X] + 1) * resolution - start_point_shift[X]) * inv_normal[X];
    }
    else
    {
        // If we decrease in a direction, we need to flip our assumptions: decrement and correct the negative sign in delta.
        // Update `dist` with how far it is to the beginning of the voxel we are in.
        dist[X]  += ((start_voxel[X] - 0) * resolution - start_point_shift[X]) * inv_normal[X];
        step[X]   = -1;
        delta[X] *= -1;
    }

    if (normal[Y] > 0)
    {
        dist[Y]  += ((start_voxel[Y] + 1) * resolution - start_point_shift[Y]) * inv_normal[Y];
    }
    else
    {
        dist[Y]  += ((start_voxel[Y] - 0) * resolution - start_point_shift[Y]) * inv_normal[Y];
        step[Y]   = -1;
        delta[Y] *= -1;
    }

    if (normal[Z] > 0)
    {
        dist[Z]  += ((start_voxel[Z] + 1) * resolution - start_point_shift[Z]) * inv_normal[Z];
    }
    else
    {
        dist[Z]  += ((start_voxel[Z] - 0) * resolution - start_point_shift[Z]) * inv_normal[Z];
        step[Z]   = -1;
        delta[Z] *= -1;
    }
}


/// @brief Calculates what voxels are hit on the ray between `sensed` and `origin`.
/// @param ray_trace[out] A trace of what voxels were hit and the distance from that voxel to the `sensed` voxel. 
/// @param sensed Sensed point, the start of the ray.
/// @param origin Origin point, the end of the ray.
/// @param properties Shared Grid Properties for the Voxel Grids begin traversed.
/// @param dist_min Minimum distance to trace along the ray, relative to the `sensed` point.
/// @param dist_min Maximum distance to trace along the ray, relative to the `sensed` point.
/// @return True if the ray intersected the Grid, this indicates that `ray_trace` has valid data to add.
inline bool get_ray_trace(std::shared_ptr<trace> ray_trace,
                          const Point& sensed, const Point& origin,
                          std::shared_ptr<const Grid::Properties> properties,
                          const float& dist_min, const float& dist_max)
{
    // Ensure any prior data is erased. Capacity is maintained.
    ray_trace->clear();

    // Adjusted min and max distances so we only trace the ray while it is within the Grid's bounds.
    float dist_min_adj = 0, dist_max_adj = 0;

    // Normalized direction for the ray (with pre-computed inverse values for AABB intersection and traversal info).
    Direction normal(0, 0, 0), inv_normal(0, 0, 0);
    float length = 0;

    vector_math::get_length_normal_and_inverse_normal(sensed, origin, length, normal, inv_normal);
    length = std::min(length, dist_max);
    
    bool valid_intersection = AABB::fast_eigen_find_bounded_intersection(properties->dimensions, sensed, inv_normal,
                                                                         dist_min, length, dist_min_adj, dist_max_adj);
    if (valid_intersection == false)
    {
        return false;
    }

    // Update the min/max distances to either the users bounds or the valid intersecting bounds.
    dist_min_adj = std::max(dist_min_adj, dist_min);
    dist_max_adj = std::min(dist_max_adj, dist_max);

    const Point sensed_adj = sensed + normal * dist_min_adj;
    Index c_idx = properties->pointToIndex(sensed_adj);

    // Direction of travel (increment or decrement) along the respective axis.
    int step[3] = {1, 1, 1};

    // The amount of distance to move one voxel length along each axis based on the ray's direction.
    float delta[3] = { properties->resolution * inv_normal[X],
                       properties->resolution * inv_normal[Y],
                       properties->resolution * inv_normal[Z]  };

    // Cumulative distance traveled along the respective axis.
    float dist[3] = {dist_min_adj, dist_min_adj, dist_min_adj};

    correct_traversal_parameters(step, delta, dist, properties->resolution, c_idx, sensed_adj, normal, inv_normal);

    /// TODO: Return to this and ensure we are recording the correct value here.
    // Pointer to the current distance moved along the ray for whatever direction we just took a step in.
    float c_dist = dist_min_adj;
    Direction res_normal = normal.cwiseAbs() * properties->resolution;

    try
    {
        // Loop runs until each direction has just passed the adjusted max distance.
        while (dist[X] <= dist_max_adj || dist[Y] <= dist_max_adj || dist[Z] <= dist_max_adj)
        {
            // Add the last step we took to the ray trace. For ease the Index is converted to a vector position.
            ray_trace->emplace_back(properties->at(c_idx), c_dist);

            // Identify which direction we have spent the least amount of time walking in.
            // Then use this direction to update for that direction.
            std::ptrdiff_t i = (dist[X] < dist[Y] && dist[X] < dist[Z]) ? X : (dist[Y] < dist[Z]) ? Y : Z;

            c_dist   += res_normal[i];
            dist[i]  += delta[i];
            c_idx[i] +=  step[i];
        }
    }
    catch (const std::out_of_range& e)
    {
        // If the traversal parameters and AABB check are correct then the algorithm should never select an
        // out of rance voxel. But checking voxel validity here does not impact performance and prevents serous
        // and potentially silent errors from occurring in a Reconstruction's Voxel Grid update steps.
        const std::string e_what(e.what());
        throw std::out_of_range("Encountered exception in get_ray_trace: " + e_what);
    }

    return true;
}


} // namespace ray_trace
} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTION_RAY_TRACING_GRID_TRAVERSAL_HPP
