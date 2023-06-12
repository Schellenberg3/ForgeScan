#ifndef FORGESCAN_SHAPE_PRIMITIVES_BOX_H
#define FORGESCAN_SHAPE_PRIMITIVES_BOX_H

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/Primitives/primative_geometry.h>

namespace ForgeScan  {
namespace Primitives {


/// @brief A simple analytical sphere object.
struct Box : public PrimitiveGeometry
{
public:
    /// @brief Dimensions ()
    const double length, width, height;

public:
    /// @brief Constructs an analytical box of the provided dimensions.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param position Translation from the origin to the center point of the box.
    Box(const double& l, const double& w, const double& h, const translation& position = Eigen::Vector3d::Zero()) :
        PrimitiveGeometry(position, getAABBbound(   std::abs(w),    std::abs(l),    std::abs(h)),
                                    getAABBbound(-1*std::abs(w), -1*std::abs(l), -1*std::abs(h))  ),
        length(l), width(w), height(h)
        { }

    /// @brief Constructs an analytical box of the provided dimensions.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param extr Extrinsic transformation from the origin to the center point of the box.
    Box(const double& l, const double& w, const double& h, const extrinsic& extr) :
        PrimitiveGeometry(extr, getAABBbound(   std::abs(w),    std::abs(l),    std::abs(h)),
                                getAABBbound(-1*std::abs(w), -1*std::abs(l), -1*std::abs(h))  ),
        length(l), width(w), height(h)
        { }

    /// @brief Constructs an analytical box of the provided dimensions.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param orientation Rotation, about the world frame, for the box's coordinate system.
    Box(const double& l, const double& w, const double& h, const rotation& orientation) :
        PrimitiveGeometry(orientation, getAABBbound(   std::abs(w),    std::abs(l),    std::abs(h)),
                                       getAABBbound(-1*std::abs(w), -1*std::abs(l), -1*std::abs(h))  ),
        length(l), width(w), height(h)
        { }

    /// @brief Determines if, and where, the line between the start and end points first intersects the geometry.
    /// @param start  Start point (position on the line when t = 0).
    /// @param end    End point (position on the line when t = 1).
    /// @param t      Intersection time (output variable). Values 0 <= t <= 1 are valid on the line segment.
    ///                - If the line DOES NOT intersect we return false with t unchanged.
    ///                - If it DOES intersect but NOT inside the bounds, then we return false with t set to the minimum
    ///                  (in magnitude) intersection time, even if this is not on the segment.
    /// @return True if the line intersects and does so in a valid region of the line.
    bool hit(const Vector3d& start, const Vector3d& end, double& t) const override final {
        return hitAABB(start, end, t);
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
};

} // Primitives
} // ForgeScan

#endif // FORGESCAN_SHAPE_PRIMITIVES_BOX_H
