#ifndef FORGESCAN_DEPTH_SENSOR_SENSOR_INTRINSICS_H
#define FORGESCAN_DEPTH_SENSOR_SENSOR_INTRINSICS_H

#include <cmath>

#include "ForgeScan/types.h"

namespace ForgeScan   {
namespace DepthSensor {
namespace Intrinsics  {


/// @brief Identification for implementations types of DepthSensor's Intrinsics.
enum class Type : int
{
    Base,
    Camera,
    Laser
};

/// @brief Abstract DepthSensor intrinsics class.
class Intrinsics
{
public:
    /// @brief Identifications for the DepthSensor type the class is intended to represent.
    Type type = Type::Base;

    /// @brief Maximum depth for the sensor and minimum depth for a sensor
    /// @note For a laser scanner type this is the maximum length for each ray.
    /// @note For a camera this is the distance to a plane normal to the sensor's principle axis that is the maximum distance for each ray.
    double d_max, d_min;

    /// @brief Upper and lower bounds for the DepthSensor's field of view. This is radial distance from the Z-axis in radians.
    /// @note Theta is the angle on the YZ-plane. Describes how 'tall' an image is.
    /// @note Phi is the angle on the XZ-plane. Describes how 'wide' an image ls.
    /// @note For a depth camera implementation the min/max values for each angle are equal in magnitude -- i.e. the field of view is symmetric.
    ///       But this is not necessarily for other implementations of DepthSensor Intrinsics.
    double theta_min, theta_max, phi_min, phi_max ;

    /// @brief Sensor size: (u, v). Number of X-pixels and number if Y-pixels.
    size_t u, v;

    /// @return Field of view in the X-direction in radians.
    /// @note This is always symmetric about the principle axis for a Camera object but is not
    ///       necessarily symmetric for other implementations of DepthSensor Intrinsics.
    double fov_x() const { return phi_max - phi_min; }

    /// @return Field of view in the Y-direction in radians.
    /// @note This is always symmetric about the principle axis for a Camera object but is not
    ///       necessarily symmetric for other implementations of DepthSensor Intrinsics.
    double fov_y() const { return theta_max - theta_min; }

    /// @brief Returns the total number of data elements in the sensor.
    /// @return Total number of data elements in the sensor.
    constexpr size_t size() const { return u * v; }

protected:
    /// @brief Constructor of the abstract intrinsics class.
    /// @param sensor_type Enumerated type ID for the sensor's implemented class.
    /// @param u Sensor dimension, number of of X-pixels.
    /// @param v Sensor dimension, number of of Y-pixels.
    /// @param d_min Minimum depth.
    /// @param d_max Maximum depth.
    /// @param theta_min Minimum FOV angle in the Sensor's YZ-plane.
    /// @param theta_max Maximum FOV angle in the Sensor's YZ-plane.
    /// @param phi_min Minimum FOV angle in the Sensor's XZ-plane.
    /// @param phi_max Maximum FOV angle in the Sensor's XZ-plane.
    Intrinsics(const Type& sensor_type, const size_t& u, const size_t& v, const double& d_min, const double& d_max,
               const double& theta_min, const double& theta_max, const double& phi_min, const double& phi_max) :
        type(sensor_type), u(u), v(v), d_min(d_min), d_max(d_max),
        theta_min(theta_min), theta_max(theta_max), phi_min(phi_min), phi_max(phi_max)
        { }
};


} // Intrinsics
} // DepthSensor
} // ForgeScan

#endif // FORGESCAN_DEPTH_SENSOR_SENSOR_INTRINSICS_H
