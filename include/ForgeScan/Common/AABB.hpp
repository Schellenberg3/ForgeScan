#ifndef FORGE_SCAN_COMMON_AABB_HPP
#define FORGE_SCAN_COMMON_AABB_HPP

#include <cstddef>
#include <cmath>

#include "ForgeScan/Common/Definitions.hpp"
#include "ForgeScan/Common/Types.hpp"


namespace forge_scan {
namespace AABB {


/// Index constants.
static constexpr std::ptrdiff_t X = 0, Y = 1, Z = 2;


/// @brief Uses a slab method to find the intersection time with the 6 planes which define an AABB
///        and returns true if ray intersects the AABB's space.
/// @details This method was adapted from excellent work done by Tavian Barnes. See:
///          https://tavianator.com/2022/ray_box_boundary.html#boundaries
/// @param dist_b1  Normalized signed distance from the start point to the first corner of the AABB.
/// @param dist_b2  Normalized signed distance from the start point to the second corner of the AABB.
/// @param tmin[out] If the intersection is valid: the time the ray enters the box.
/// @param tmax[out] If the intersection is valid: the time the ray exits the box.
/// @return  True if the intersection is valid: `tmin<=tmax`.
/// @warning This function is a helper for `find_bounded_intersection` and
///          `find_zero_bounded_intersection`. It should not be called on its own.
inline bool find_intersection(const Eigen::Vector3f& dist_b1,
                              const Eigen::Vector3f& dist_b2,
                              float& tmin,
                              float& tmax)
{
    tmin = -1 * INFINITY;
    tmax =      INFINITY;

    tmin = std::min(std::max(dist_b1[X], tmin), std::max(dist_b2[X], tmin));
    tmax = std::max(std::min(dist_b1[X], tmax), std::min(dist_b2[X], tmax));

    tmin = std::min(std::max(dist_b1[Y], tmin), std::max(dist_b2[Y], tmin));
    tmax = std::max(std::min(dist_b1[Y], tmax), std::min(dist_b2[Y], tmax));

    tmin = std::min(std::max(dist_b1[Z], tmin), std::max(dist_b2[Z], tmin));
    tmax = std::max(std::min(dist_b1[Z], tmax), std::min(dist_b2[Z], tmax));

    return tmin <= tmax;
}


/// @brief Determines if and where a time-parameterized ray intersects an axis-aligned bounding
///        box (AABB). The outputs `tmin` and `tmax` may either be scalar multipliers of the ray
///        or distances in the same units of the ray.
/// @details This method was adapted from work done by Tavian Barnes. See:
///          https://tavianator.com/2022/ray_box_boundary.html
/// @param bound1  The first corner of the AABB.
/// @param bound2  The second corner of the AABB.
/// @param start   Starting point of the ray.
/// @param inv_ray Element-wise inverse of the ray. This value may either be from the full ray 
///                (which makes the outputs scalar multipliers for the ray) or its unit-direction
///                (which makes the outputs distances to the box in the same units of the ray)
/// @param tmin_bound Lower bound time for the intersection to be valid.
/// @param tmax_bound Upper bound time for the intersection to be valid.
/// @param tmin[out] If the intersection is valid: the time the ray enters the box.
/// @param tmax[out] If the intersection is valid: the time the ray exits the box.
/// @return True if the intersection is valid: `tmin<=tmax` and `tmin<=max_bound` and
///         `tmin_bound<=tmax`.
inline bool find_bounded_intersection(const Point& bound1,     const Point& bound2,
                                        const Point& start,      const Direction& inv_ray,
                                        const float& tmin_bound, const float& tmax_bound,
                                        float& tmin,             float& tmax)
{
    const Eigen::Vector3f dist_b1 = (bound1 - start).array() * inv_ray.array();
    const Eigen::Vector3f dist_b2 = (bound2 - start).array() * inv_ray.array();

    bool ray_intersects = find_intersection(dist_b1, dist_b2, tmin, tmax);
    return ray_intersects && ( (tmin <= tmax_bound) && (tmin_bound <= tmax) );
}


/// @brief Determines if and where a time-parameterized ray intersects an axis-aligned bounding
///        box (AABB). The outputs `tmin` and `tmax` may either be scalar multipliers of the ray
///        or distances in the same units of the ray.
/// @param bound   The corner of the AABB. The other corner is implicitly at (0, 0, 0).
/// @param start   Starting point of the ray.
/// @param inv_ray Element-wise inverse of the ray. This value may either be from the full ray 
///                (which makes the outputs scalar multipliers for the ray) or its unit-direction
///                (which makes the outputs distances to the box in the same units of the ray)
/// @param tmin_bound Lower bound time for the intersection to be valid.
/// @param tmax_bound Upper bound time for the intersection to be valid.
/// @param tmin[out] If the intersection is valid: the time the ray enters the box.
/// @param tmax[out] If the intersection is valid: the time the ray exits the box.
/// @return True if the intersection is valid: `tmin<=tmax`.
inline bool find_zero_bounded_intersection(const Point& bound,
                                             const Point& start,      const Direction& inv_ray,
                                             const float& tmin_bound, const float& tmax_bound,
                                             float& tmin,             float& tmax)
{
    const Eigen::Vector3f dist_b1 = -1 * start.array() * inv_ray.array();
    const Eigen::Vector3f dist_b2 = (bound - start).array() * inv_ray.array();

    bool ray_intersects = find_intersection(dist_b1, dist_b2, tmin, tmax);
    return ray_intersects && ( (tmin <= tmax_bound) && (tmin_bound <= tmax) );
}


// ********************************************************************************************* //
// *                                    FAST EIGEN METHODS                                     * //
// * In theory the above methods should be faster but these original methods I wrote seem to   * //
// * be significantly faster for the Ray Trace code. However, they fail when creating an image * //
// * from a Scene. I've not determined why they are successful when ray tracing but fail in    * //
// * image generation. This is something I will return to. But for now I will just use both.   * //
// ********************************************************************************************* //



/// @brief  A more generic implementation for AABB bounds checking.
/// @return True if the ray will intersect.
/// @warning This function is a helper for `fast_eigen_find_intersection` and
///          `fast_eigen_find_bounded_intersection`. It should not be called on its own.
inline bool fast_eigen_intersection_check(const Eigen::Vector3f& min_dist,
                                          const Eigen::Vector3f& max_dist,
                                          const Direction& inv_normal,
                                          const float& s_dist, const float& e_dist,
                                          float& s_dist_bound, float& e_dist_bound)
{
    s_dist_bound = inv_normal[X] >= 0 ? min_dist[X] :  max_dist[X];
    e_dist_bound = inv_normal[X] >= 0 ? max_dist[X] :  min_dist[X];

    float dist_min_y = inv_normal[Y] >= 0 ? min_dist[Y] : max_dist[Y];
    float dist_max_y = inv_normal[Y] >= 0 ? max_dist[Y] : min_dist[Y];

    bool bound_check_1 = !(s_dist_bound > dist_max_y || dist_min_y > e_dist_bound);

    s_dist_bound = dist_min_y > s_dist_bound ? dist_min_y : s_dist_bound;
    e_dist_bound = dist_max_y < e_dist_bound ? dist_min_y : e_dist_bound;

    float dist_min_z = inv_normal[Z] >= 0 ? min_dist[Z] : max_dist[Z];
    float dist_max_z = inv_normal[Z] >= 0 ? max_dist[Z] : min_dist[Z];

    bool bound_check_2 = !(s_dist_bound > dist_max_z || dist_min_z > e_dist_bound);

    s_dist_bound = dist_min_z > s_dist_bound ? dist_min_z : s_dist_bound;
    e_dist_bound = dist_max_z < e_dist_bound ? dist_max_z : e_dist_bound;

    return bound_check_1 && bound_check_2 && s_dist_bound <= e_dist_bound &&
           s_dist_bound <= e_dist && s_dist <= e_dist_bound;
}


/// @brief Checks that a ray intersects an zero-bounded Axis-Aligned Bounding Box (AABB) over the
///        specified segment.
/// @param lower_bound  Lower bound of the AABB.
/// @param upper_bound  Upper bound of the AABB.
/// @param start        Start location of the ray to check. Distances are relative to this point
///                     along the direction.
/// @param inv_normal The element-wise inverse of the ray's direction vector. The inverse is
///                   computed for other ray tracing needs, passing it here prevents re-calculation.
/// @param s_dist Beginning of the intersection segment. Distance from the ray's origin along the
///               ray's direction.
/// @param e_dist End of the intersection segment. Distance from the ray's origin along the ray's
///               direction.
/// @param s_dist_bound[out] Adjusted beginning of the segment. Distance from the ray's origin
///                          along the ray's direction to when the ray enters the AABB.
/// @param e_dist_bound[out] Adjusted end of the segment. Distance from the ray's origin along
///                          the ray's direction to when the ray exits the AABB.
/// @return  True if the ray intersects the AABB over the specified segment. False else.
/// @warning - If this returns false then `s_dist_bound` and `e_dist_bound` should not be used.
/// @warning - This assumes that `inv_normal` is normalized but does neither checks nor performs
///            this operation.
/// @warning - This requires `e_dist` to be greater `s_dist` but does not check. The function will
///            incorrectly return false if this is wrong, even if the segment technically intersects.
/// @warning - This assumes that all values in `upper_bound` are positive, but does not check.
inline bool fast_eigen_find_intersection(const Point& lower_bound, const Point& upper_bound,
                                         const Point& start,       const Direction& inv_normal,
                                         const float& s_dist,      const float& e_dist,
                                         float& s_dist_bound,      float& e_dist_bound)
{
    const Eigen::Vector3f min_dist = (lower_bound - start).array() * inv_normal.array();
    const Eigen::Vector3f max_dist = (upper_bound - start).array() * inv_normal.array();

    return fast_eigen_intersection_check(min_dist, max_dist, inv_normal, s_dist, e_dist, s_dist_bound, e_dist_bound);
}


/// @brief Checks that a ray intersects an zero-bounded Axis-Aligned Bounding Box (AABB) over the
///        specified segment.
/// @param bound  Upper bound of the AABB.
/// @param start  Start location of the ray to check. Distances are relative to this point along
///               the direction.
/// @param inv_normal The element-wise inverse of the ray's direction vector. The inverse is
///                   computed for other ray tracing needs, passing it here prevents re-calculation.
/// @param s_dist Beginning of the intersection segment. Distance from the ray's origin along the
///               ray's direction.
/// @param e_dist End of the intersection segment. Distance from the ray's origin along the ray's
///               direction.
/// @param s_dist_bound[out] Adjusted beginning of the segment. Distance from the ray's origin
///                          along the ray's direction to when the ray enters the AABB.
/// @param e_dist_bound[out] Adjusted end of the segment. Distance from the ray's origin along
///                          the ray's direction to when the ray exits the AABB.
/// @return  True if the ray intersects the AABB over the specified segment. False else.
/// @warning - If this returns false then `s_dist_bound` and `e_dist_bound` should not be used.
/// @warning - This assumes that `inv_normal` is normalized but does neither checks nor performs
///            this operation.
/// @warning - This requires `e_dist` to be greater `s_dist` but does not check. The function will
///            incorrectly return false if this is wrong, even if the segment technically intersects.
/// @warning - This assumes that all values in `upper_bound` are positive, but does not check.
inline bool fast_eigen_find_bounded_intersection(const Point& bound,
                                                 const Point& start,  const Direction& inv_normal,
                                                 const float& s_dist, const float& e_dist,
                                                 float& s_dist_bound, float& e_dist_bound)
{
    const Eigen::Vector3f min_dist =     -1 * start.array()  * inv_normal.array();
    const Eigen::Vector3f max_dist = (bound - start).array() * inv_normal.array();

    return fast_eigen_intersection_check(min_dist, max_dist, inv_normal, s_dist, e_dist, s_dist_bound, e_dist_bound);
}


} // namespace AABB
} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_AABB_HPP
