#ifndef FORGE_SCAN_COMMON_AABB_HPP
#define FORGE_SCAN_COMMON_AABB_HPP

#include <cstddef>
#include <cmath>

#include "ForgeScan/Common/Definitions.hpp"
#include "ForgeScan/Common/Types.hpp"


namespace forge_scan {


/// @brief Provides methods for finding when and where a ray intersects an Axis Aligned Bounding Box (AABB).
/// @note  While technically a `struct`, this only has static functions so it acts more like a
///        namespace with priviledged access to Camera's internal information.
struct AABB 
{
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
    /// @param [out] tmin If the intersection is valid: the time the ray enters the box.
    /// @param [out] tmax If the intersection is valid: the time the ray exits the box.
    /// @return True if the intersection is valid: `tmin<=tmax` and `tmin<=max_bound` and
    ///         `tmin_bound<=tmax`.
    static bool find_bounded_intersection(const Point& bound1,     const Point& bound2,
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
    /// @param [out] tmin If the intersection is valid: the time the ray enters the box.
    /// @param [out] tmax If the intersection is valid: the time the ray exits the box.
    /// @return True if the intersection is valid: `tmin<=tmax`.
    static bool find_zero_bounded_intersection(const Point& bound,
                                            const Point& start,      const Direction& inv_ray,
                                            const float& tmin_bound, const float& tmax_bound,
                                            float& tmin,             float& tmax)
    {
        const Eigen::Vector3f dist_b1 =     -1 * start.array()  * inv_ray.array();
        const Eigen::Vector3f dist_b2 = (bound - start).array() * inv_ray.array();

        bool ray_intersects = find_intersection(dist_b1, dist_b2, tmin, tmax);
        return ray_intersects && ( (tmin <= tmax_bound) && (tmin_bound <= tmax) );
    }

private:
    /// @brief Vector indexing helpers.
    static constexpr std::ptrdiff_t X = 0, Y = 1, Z = 2;


    /// @brief Uses a slab method to find the intersection time with the 6 planes which define an AABB
    ///        and returns true if ray intersects the AABB's space.
    /// @details This method was adapted from excellent work done by Tavian Barnes. See:
    ///          https://tavianator.com/2022/ray_box_boundary.html#boundaries
    /// @param dist_b1  Normalized signed distance from the start point to the first corner of the AABB.
    /// @param dist_b2  Normalized signed distance from the start point to the second corner of the AABB.
    /// @param [out] tmin If the intersection is valid: the time the ray enters the box.
    /// @param [out] tmax If the intersection is valid: the time the ray exits the box.
    /// @return  True if the intersection is valid: `tmin<=tmax`.
    static bool find_intersection(const Eigen::Vector3f& dist_b1,
                                  const Eigen::Vector3f& dist_b2,
                                  float& tmin,
                                  float& tmax)
    {
        tmin = -1 * INFINITY;
        tmax =      INFINITY;

        tmin = std::fmin(std::fmax(dist_b1[X], tmin), std::fmax(dist_b2[X], tmin));
        tmax = std::fmax(std::fmin(dist_b1[X], tmax), std::fmin(dist_b2[X], tmax));

        tmin = std::fmin(std::fmax(dist_b1[Y], tmin), std::fmax(dist_b2[Y], tmin));
        tmax = std::fmax(std::fmin(dist_b1[Y], tmax), std::fmin(dist_b2[Y], tmax));

        tmin = std::fmin(std::fmax(dist_b1[Z], tmin), std::fmax(dist_b2[Z], tmin));
        tmax = std::fmax(std::fmin(dist_b1[Z], tmax), std::fmin(dist_b2[Z], tmax));

        return tmin <= tmax;
    }
};


} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_AABB_HPP
