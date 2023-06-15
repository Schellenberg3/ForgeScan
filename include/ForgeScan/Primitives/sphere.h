#ifndef FORGESCAN_SHAPE_PRIMITIVES_SPHERE_H
#define FORGESCAN_SHAPE_PRIMITIVES_SPHERE_H

#include "ForgeScan/Primitives/primative.h"


namespace ForgeScan  {
namespace Primitives {


/// @brief A simple analytical sphere object.
struct Sphere : public Primitive
{
public:
    /// @brief Sphere radius in world units.
    const double radius;

public:
    /// @brief Constructs an analytical sphere with the given radius at the specified position
    /// @param radius Radius value for the sphere. Default is 1 unit.
    /// @param center Location of the sphere's center point in world coordinate.
    ///               Default places the sphere at the origin
    /// @note This takes the absolute value of the provided radius value.
    Sphere(const double& radius = 1, const point& center = Eigen::Vector3d::Zero()) :
        Primitive(center, getAABBbound(std::abs(radius)), getAABBbound(-1 * std::abs(radius))),
        radius(std::abs(radius))
        { }

    /// @brief Determines if, and where, the line between the start and end points first intersects the geometry.
    /// @param start  Start point (position on the line when t = 0).
    /// @param end    End point (position on the line when t = 1).
    /// @param t      Intersection time (output variable). Values 0 <= t <= 1 are valid on the line segment.
    ///                - If the line DOES NOT intersect we return false with t unchanged.
    ///                - If it DOES intersect but NOT inside the bounds, then we return false with t set to the minimum
    ///                  (in magnitude) intersection time, even if this is not on the segment.
    /// @return True if the line intersects and does so in a valid region of the line.
    bool hit(const Vector3d& start, const Vector3d& end, double& t) const override final
    {
        double unused_intersection_time;
        if ( !hitAABB(start, end, unused_intersection_time) ) return false;

        /// Adapted from https://stackoverflow.com/questions/6533856 with a partial quadratic solver for only the
        /// real-valued solutions of the intersection. Also, see; http://paulbourke.net/geometry/circlesphere/

        /// Quadratic equation for intersection:
        ///     0 = A*(x*x) + B*x + C
        const point center = extr.translation();
        double R2 = radius * radius;
        double A = ((start - end   ).array().pow(2)).sum();
        double C = ((start - center).array().pow(2)).sum() - R2;
        double B = ((end   - center).array().pow(2)).sum() - A - C - R2;

        /// Find quadratic equation determinant. Early exit if negative (complex solutions mean no intersection).
        double D = B*B - 4*A*C;
        if (D < 0) return false;

        /// Pre-calculations help us optimize the quadratic formula. And checking the sign of B lets
        /// us utilize an numerically stable form in which only addition OR subtraction is  required.
        ///     D = sqrt(B*B - 4*A*C)
        ///     if B < 0
        ///         X_1 = (-B + D) / 2*A
        ///         X_2 = 2*C / (-B + D)
        ///         (Leads to adding two positives)
        ///     if B >= 0
        ///         X_1 = (-B - D) / 2*A
        ///         X_2 = 2*C / (-B - D)
        /// In short, the first case lets us add two positives and the second lets us subtract two negatives. This
        /// is ideal as it avoids any case where we subtract quantities with the same sign. In cases where these values
        /// are similar in magnitude (for this case, when 4*A*C is small) this leads to imprecision in rounding.
        /// For details on the numeric stability see: https://people.csail.mit.edu/bkph/articles/Quadratics.pdf

        /// Both cases require the following values which we may pre-compute
        D = std::sqrt(D);
        A *= 2;
        C *= 2;
        B *= -1;

        if (B > 0) {
            B += D;
        } else {
            B -= D;
        }
        t = C / B;
        double x = B / A;

        if (t >= 0) {
            /// Find minimum if both are positive. Else, leave t unchanged as the other solution is non-positive.
            if (x > 0) t = std::min(t, x);
        } else if (t <= 0) {
            /// Find maximum if both are negative. Else, set t to the other solution, which must be non-negative.
            t = x < 0 ? std::max(t, x) : x;
        }
        /// For the case t == 0 the answers are the same so no comparison is needed.

        /// In this, 0 <= t <= 1 indicates a point between the two values of interest.
        return ( 0 <= t && t <= 1 );
    }

    /// @brief Calculates the shortest signed distance between the point and the Sphere's surface.
    /// @param input Point in space.
    /// @param extr  Frame which the point is in.
    /// @return The shortest distance between the point and the surface with negative distances being inside the Sphere.
    double getSignedDistance(const point& input, const extrinsic& extr) const override final {
        const point input_this = toThisFromOther(input, extr);
        return input_this.norm() - radius;
    }

private:
    /// @brief Constructor helper for generating a sphere's AABB bounds.
    /// @param radius Radius of the sphere.
    ///               Pass as positive for the upper bound. Pass as negative for the lower bound.
    /// @return Axis-aligned bounding box point for the upper or lower, depending on the radius' sign.
    static point getAABBbound(const double& radius) { return point(radius, radius, radius); }
};


} // namespace Primitives
} // namespace ForgeScan

#endif // FORGESCAN_SHAPE_PRIMITIVES_SPHERE_H
