#ifndef FORGESCAN_SHAPE_PRIMITIVES_PRIMITIVE_H
#define FORGESCAN_SHAPE_PRIMITIVES_PRIMITIVE_H

#include "ForgeScan/types.h"
#include "ForgeScan/entity.h"


namespace ForgeScan  {
namespace Primitives {


/// @brief Base class for all primitive geometry types.
struct Primitive : Entity
{
public:
    /// @brief Bounding box limits for the geometric primitive.
    const point upperAABBbound, lowerAABBbound;

public:
    /// @brief Determines if, and where, the line between the start and end points first intersects the geometry.
    /// @param start  Start point (position on the line when t = 0).
    /// @param end    End point (position on the line when t = 1).
    /// @param t      Intersection time (output variable). Values 0 <= t <= 1 are valid on the line segment.
    ///                - If the line DOES NOT intersect we return false with t unchanged.
    ///                - If it DOES intersect but NOT inside the bounds, then we return false with t set to the minimum
    ///                  (in magnitude) intersection time, even if this is not on the segment.
    /// @return True if the line intersects and does so in a valid region of the line.
    virtual bool hit(const point& start, const point& end, double& t) const = 0;

    /// @brief Calculates the shortest signed distance between the point and the Primitive's surface.
    /// @param input Point in space.
    /// @param extr  Frame which the point is in.
    /// @return The shortest distance between the point and the surface with negative distances being inside the Primitive.
    virtual double getSignedDistance(const point& input, const extrinsic& extr) const = 0;

    /// @brief Checks if a point is within the axis-aligned bounding box (AABB) for the Primitive.
    /// @param input Point to check.
    /// @return True if the point is between the upper and lower AABB bound (or on one of them). False else.
    bool insideBounds(const point& input) const
        { return (input.array() >= lowerAABBbound.array()).all() && (input.array() <= upperAABBbound.array()).all(); }

protected:
    /// @brief Constructs the generic geometric primitive at the world origin.
    Primitive(const point& upperAABBbound, const point& lowerAABBbound) :
        Entity(), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
        { }

    /// @brief Constructs the generic geometric primitive.
    /// @param extr Initial pose for the entity.
    Primitive(const extrinsic& extr, const point& upperAABBbound, const point& lowerAABBbound) :
        Entity(extr), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
        { }

    /// @brief Constructs the generic geometric primitive at the position, no rotation.
    /// @param position Initial position for the entity.
    Primitive(const translation& position, const point& upperAABBbound, const point& lowerAABBbound) :
        Entity(position), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
        { }

    /// @brief Constructs the generic geometric primitive at the world origin with the given rotation.
    /// @param orientation Initial rotation for the entity.
    Primitive(const rotation& orientation, const point& upperAABBbound, const point& lowerAABBbound) :
        Entity(orientation), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
        { }

    /// @brief Relatively quick check for bounding box intersection of a geometric primitive. Prevents needless checks for some rays.
    /// @param start Starting point of the ray, relative to the primitive's frame.
    /// @param end   Ending point of the ray, relative to the primitive's frame.
    /// @param t     Adjusted start intersection time (output variable). Describes when the line will first intersect the AABB.
    ///              Values 0 <= t <= 1 are valid on the line segment.
    /// @return True if the ray has any intersection with the primitive's bounding box.
    /// @warning The output variable, `t`, is valid when this function returns true.
    ///          Otherwise it does not describe an intersection and should not be trusted.
    bool hitAABB(const point& start, const point& end, double& t) const
    {
        double t_min_y = 0, t_max_y = 0, t_min_z = 0, t_max_z = 0, ts_adj = 0, te_adj = 0;

        const Vector3d inverse_ray = (end - start).cwiseInverse();

        /// Minimum distance = ([lower bounds] - [ray origin]) / [direction]
        /// Assumed to be zero-bounded on the minimum edge.
        const Vector3d min_dist = (lowerAABBbound - start).array() * inverse_ray.array();

        /// Maximum distance = ([Upper bounds] - [ray origin]) / [direction]
        const Vector3d max_dist = (upperAABBbound - start).array() * inverse_ray.array();

        if (inverse_ray[0] >= 0)
        {
            ts_adj = min_dist[0];
            te_adj = max_dist[0];
        } else {
            ts_adj = max_dist[0];
            te_adj = min_dist[0];
        }

        if (inverse_ray[1] >= 0)
        {
            t_min_y = min_dist[1];
            t_max_y = max_dist[1];
        } else {
            t_min_y = max_dist[1];
            t_max_y = min_dist[1];
        }

        if (ts_adj > t_max_y || t_min_y > te_adj) return false;

        if (t_min_y > ts_adj) ts_adj = t_min_y;
        if (t_max_y < te_adj) te_adj = t_max_y;

        if (inverse_ray[2] >= 0)
        {
            t_min_z = min_dist[2];
            t_max_z = max_dist[2];
        } else {
            t_min_z = max_dist[2];
            t_max_z = min_dist[2];
        }

        if (ts_adj > t_max_z || t_min_z > te_adj) return false;

        if (t_min_z > ts_adj) ts_adj = t_min_z;
        if (t_max_z < te_adj) te_adj = t_max_z;

        t = ts_adj;
        return (ts_adj < 1 && 0 < te_adj);
    }
};


} // namespace Primitives
} // namespace ForgeScan

#endif // FORGESCAN_SHAPE_PRIMITIVES_PRIMITIVE_H
