#ifndef FORGESCAN_SHAPE_PRIMITIVES_PRIMITIVE_GEOMETRY_H
#define FORGESCAN_SHAPE_PRIMITIVES_PRIMITIVE_GEOMETRY_H


#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/depth_sensor.h>


namespace ForgeScan {
namespace Primitives {

struct PrimitiveGeometry; // Forward declaration for the typedef

/// @brief A collection of PrimitiveGeometry objects which are imaged together in the same scene.
typedef std::vector<PrimitiveGeometry*> Scene;

/// @brief Base class for all primitive geometry types.
struct PrimitiveGeometry : ForgeScanEntity
{
public:
    void image(DepthSensor::BaseDepthSensor& sensor, const bool& reset_first = true)
    {
        /// If requested, reset all points to their maximum depth before imaging.
        if (reset_first) { sensor.resetDepth(); }

        /// Get the sensor's position and viewed points relative to the geometric primitive.
        /// The hit methods generally don't depend on this, they will work as long both the start point and end points are
        /// in the same reference frame, but this is needed for the AABB checks.
        point start = sensor.extr.translation();           // world frame
        fromWorldToThis(start);

        point_list end_points = sensor.getAllPositions();  // sensor frame
        fromOtherToThis(sensor, end_points);

        double t = 1;
        for (size_t i = 0, num_pts = sensor.intr->size(); i < num_pts; ++i)
        {
            /// Run intersection search; if there is a valid hit then we scale the depth value by t, the returned time. 
            /// All we do is scale the depth value.
            if ( hit(start, end_points.col(i), t) ) {
                sensor(i) *= t;
            }
        }
    }


    /// @brief Determines if, and where, the line between the start and end points first intersects the geometry.
    /// @param start  Start point (position on the line when t = 0).
    /// @param end    End point (position on the line when t = 1).
    /// @param t      Intersection time (output variable). Values 0 <= t <= 1 are valid on the line segment.
    ///                - If the line DOES NOT intersect we return false with t unchanged.
    ///                - If it DOES intersect but NOT inside the bounds, then we return false with t set to the minimum
    ///                  (in magnitude) intersection time, even if this is not on the segment.
    /// @return True if the line intersects and does so in a valid region of the line.
    virtual bool hit(const point& start, const point& end, double& t) const = 0;

protected:
    /// @brief Bounding box limits for the geometric primitive.
    const point upperAABBbound, lowerAABBbound;

protected:
    /// @brief Constructs the generic geometric primitive at the world origin.
    PrimitiveGeometry(const point& upperAABBbound, const point& lowerAABBbound) :
        ForgeScanEntity(), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
        { }

    /// @brief Constructs the generic geometric primitive.
    /// @param extr Initial pose for the entity.
    PrimitiveGeometry(const extrinsic& extr, const point& upperAABBbound, const point& lowerAABBbound) :
        ForgeScanEntity(extr), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
        { }

    /// @brief Constructs the generic geometric primitive at the position, no rotation.
    /// @param position Initial position for the entity.
    PrimitiveGeometry(const translation& position, const point& upperAABBbound, const point& lowerAABBbound) :
        ForgeScanEntity(position), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
        { }

    /// @brief Constructs the generic geometric primitive at the world origin with the given rotation.
    /// @param orientation Initial rotation for the entity.
    PrimitiveGeometry(const rotation& orientation, const point& upperAABBbound, const point& lowerAABBbound) :
        ForgeScanEntity(orientation), upperAABBbound(upperAABBbound), lowerAABBbound(lowerAABBbound)
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

} // Primitives
} // ForgeScan

#endif // FORGESCAN_SHAPE_PRIMITIVES_PRIMITIVE_GEOMETRY_H
