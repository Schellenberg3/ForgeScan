#ifndef FORGESCAN_SHAPE_PRIMITIVES_BOX_H
#define FORGESCAN_SHAPE_PRIMITIVES_BOX_H

#include "ForgeScan/Primitives/primative.h"


namespace ForgeScan  {
namespace Primitives {


/// @brief A simple analytical sphere object.
struct Box : public Primitive
{
public:
    /// @brief Dimensions of the box: length in X-direction, width in Y-direction, and height it Z-direction.
    const double length, width, height;

public:
    /// @brief Constructs an analytical box of the provided dimensions.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param position Translation from the origin to the center point of the box.
    Box(const double& l, const double& w, const double& h, const translation& position = Eigen::Vector3d::Zero()) :
        Primitive(position, getAABBbound(   std::abs(l),    std::abs(w),    std::abs(h)),
                            getAABBbound(-1*std::abs(l), -1*std::abs(w), -1*std::abs(h))),
        length(l), width(w), height(h)
        { }

    /// @brief Constructs an analytical box of the provided dimensions.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param extr Extrinsic transformation from the origin to the center point of the box.
    Box(const double& l, const double& w, const double& h, const extrinsic& extr) :
        Primitive(extr, getAABBbound(   std::abs(l),    std::abs(w),    std::abs(h)),
                        getAABBbound(-1*std::abs(l), -1*std::abs(w), -1*std::abs(h))),
        length(l), width(w), height(h)
        { }

    /// @brief Constructs an analytical box of the provided dimensions.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param orientation Rotation, about the world frame, for the box's coordinate system.
    Box(const double& l, const double& w, const double& h, const rotation& orientation) :
        Primitive(orientation, getAABBbound(   std::abs(l),    std::abs(w),    std::abs(h)),
                               getAABBbound(-1*std::abs(l), -1*std::abs(w), -1*std::abs(h))),
        length(l), width(w), height(h)
        { }

    /// @brief Determines if, and where, the line between the start and end points first intersects the geometry.
    /// @param start  Start point (position on the line when t = 0), relative to the Box's frame.
    /// @param end    End point (position on the line when t = 1), relative to the Box's frame.
    /// @param t      Intersection time (output variable). Values 0 <= t <= 1 are valid on the line segment.
    ///                - If the line DOES NOT intersect we return false with t unchanged.
    ///                - If it DOES intersect but NOT inside the bounds, then we return false with t set to the minimum
    ///                  (in magnitude) intersection time, even if this is not on the segment.
    /// @return True if the line intersects and does so in a valid region of the line.
    bool hit(const Vector3d& start, const Vector3d& end, double& t) const override final {
        return hitAABB(start, end, t);
    }

    /// @brief Calculates the shortest signed distance between the point and the Box's surface.
    /// @param input Point in space.
    /// @param extr  Frame which the point is in.
    /// @return The shortest distance between the point and the surface with negative distances being inside the Box.
    double getSignedDistance(const point& input, const extrinsic& extr) const override final {
        const point input_this = toThisFromOther(input, extr);
        if (insideBounds(input_this)) {
            return getSignedDistanceInside(input_this);
        }
        return getSignedDistanceOutside(input_this);
    }

private:
    /// @brief Constructor helper for generating a box's AABB bounds.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @return Axis-aligned bounding box point for the upper or lower, depending on the sign of the dimension.
    static point getAABBbound(const double& l, const double& w, const double& h) {
        return point(l, w, h).operator*=(0.5);
    }

    /// @brief Calculates the signed distance for the case where a point is outside of the Box.
    /// @param input_this Point, relative to the Box's reference frame.
    /// @return Shortest distance from the box to that point. Always non-negative.
    double getSignedDistanceOutside(const point& input_this) const {
        double dx = 0, dy = 0, dz = 0;
        dx = std::max(std::max(lowerAABBbound[0] - input_this[0], 0.0), input_this[0] - upperAABBbound[0]);
        dy = std::max(std::max(lowerAABBbound[1] - input_this[1], 0.0), input_this[1] - upperAABBbound[1]);
        dz = std::max(std::max(lowerAABBbound[2] - input_this[2], 0.0), input_this[2] - upperAABBbound[2]);
        return std::hypot(dx, dy, dz);
    }

    /// @brief Calculates the signed distance for the case where a point is inside of the Box.
    /// @param input_this Point, relative to the Box's reference frame.
    /// @return Shortest distance from the box to that point. Always non-positive.
    double getSignedDistanceInside(const point& input_this) const {
        double minx = 0, miny = 0, minz = 0;
        minx = std::min(input_this[0] - lowerAABBbound[0], upperAABBbound[0] - input_this[0]);
        miny = std::min(input_this[1] - lowerAABBbound[1], upperAABBbound[1] - input_this[1]);
        minz = std::min(input_this[2] - lowerAABBbound[2], upperAABBbound[2] - input_this[2]);
        return -1 * std::min(std::min(minx, miny), minz);
    }
};


} // namespace Primitives
} // namespace ForgeScan

#endif // FORGESCAN_SHAPE_PRIMITIVES_BOX_H
