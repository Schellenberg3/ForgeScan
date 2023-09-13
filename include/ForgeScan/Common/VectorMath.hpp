#ifndef FORGE_SCAN_COMMON_VECTOR_MATH_HPP
#define FORGE_SCAN_COMMON_VECTOR_MATH_HPP

#include "ForgeScan/Common/Types.hpp"


namespace forge_scan {
namespace vector_math {


/// @brief Calculates and returns the normal and length for the ray between the two points.
/// @param start Start of the ray.
/// @param end   End of the ray.
/// @param [out] length Length between the start and end Point.
/// @param [out] normal The direction of the vector, normalized.
inline void get_length_and_normal(const Point& start, const Point& end,
                                 float& length, Direction& normal)
{
    const Ray span = end - start;
    length = span.norm();
    normal = span / length;
}


/// @brief Calculates and returns the normal and length for the ray between the two points.
/// @param start Start of the ray.
/// @param end   End of the ray.
/// @param [out] length Length between the start and end Point.
/// @param [out] normal The direction of the vector, normalized.
/// @param [out] inv_normal Elementwise inverse of the normal.
/// @note  This function is typically used for ray-tracing-related code where both the normal
///        and a pre-computed inverse normal are useful.
inline void get_length_normal_and_inverse_normal(const Point& start, const Point& end,
                                                 float& length, Direction& normal, Direction& inv_normal)
{
    get_length_and_normal(start, end, length, normal);
    inv_normal = normal.cwiseInverse();
}


/// @brief Calculates the rotation needed to point the Z-Axis of an reference frame to the target point.
/// @param position Cartesian position of the reference frame.
/// @param target Target to point the axis at.
/// @param axis Axis or vector in the reference frame to point at the target point.
/// @return Quaternion rotation to apply.
inline Eigen::Quaternionf get_rotation_to_orient_to_axis(const Point& position, const Point& target, const Ray& axis = Ray::UnitZ())
{
    return Eigen::Quaternionf().setFromTwoVectors(axis, target - position);
}


/// @brief Calculates the rotation needed to point the Z-Axis of an reference frame to the target point.
/// @param extr Extrinsic matrix of the reference frame.
/// @param target Target to point the axis at.
/// @param axis Axis of the .
/// @return Quaternion rotation to apply.
inline Eigen::Quaternionf get_rotation_to_orient_to_axis(const Extrinsic& extr, const Point& target, const Ray& axis = Ray::UnitZ())
{
    return get_rotation_to_orient_to_axis(extr.translation(), target, axis);
}


/// @brief Calculates the rotation needed to point the Z-Axis of an reference frame to the target point.
/// @param position Cartesian position of the reference frame.
/// @param target Target to point the z-axis at.
/// @return Quaternion rotation to apply.
inline Eigen::Quaternionf get_rotation_to_orient_z_axis(const Point& position, const Point& target)
{
    // Unit vector in the Z-axis.
    const static Ray principle_axis(0, 0, 1);
    return get_rotation_to_orient_to_axis(position, target, principle_axis);
}


/// @brief Calculates the rotation needed to point the Z-Axis of an reference frame to the target point.
/// @param extr Extrinsic matrix of the reference frame.
/// @param target Target to point the z-axis at.
/// @return Quaternion rotation to apply.
inline Eigen::Quaternionf get_rotation_to_orient_z_axis(const Extrinsic& extr, const Point& target)
{
    return get_rotation_to_orient_z_axis(extr.translation(), target);
}


/// @brief Templated function for converting from spherical coordinates to cartesian ones.
/// @param r Radius.
/// @param theta Angle around the positive X-axis. [0, 2*PI) Radians.
/// @param phi   Angle angle from positive Z-axis. [0, PI)   Radians.
inline Point spherical_to_cartesian(const float& r, const float& theta, const float& phi)
{
    Point output(0, 0, 0);
    output.x() = r * std::sin(theta) * std::sin(phi);
    output.y() = r * std::cos(theta) * std::sin(phi);
    output.z() = r * std::cos(phi);
    return output;
}


} // namespace vector_math
} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_VECTOR_MATH_HPP
