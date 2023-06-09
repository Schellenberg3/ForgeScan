#ifndef FORGESCAN_SENSOR_INTRINSICS_H
#define FORGESCAN_SENSOR_INTRINSICS_H

#include <cmath>


namespace ForgeScan {
namespace Intrinsics {

/// @brief Type identification for implementations of BaseDepthSensor.
enum class DepthSensorType : int
{
    Base,
    DepthCamera,
    LaserScanner,
    ConicRandomLaserScanner
};

class BaseDepthSensor
{
public:
    /// @brief Identifications for the DepthSensor type the class is intended to represent.
    DepthSensorType type = DepthSensorType::Base;

    /// @brief Maximum depth for the sensor and minimum depth for a sensor
    /// @note For a laser scanner type this is the maximum length for each ray.
    /// @note For a camera this is the distance to a plane normal to the sensor's principle axis that is the maximum distance for each ray.
    double d_max, d_min;

    /// @brief Upper and lower bounds for the DepthSensor's field of view. This is radial distance from the Z-axis in radians.
    /// @note Theta is the angle on the YZ-plane. Describes how 'tall' an image is.
    /// @note Phi is the angle on the XZ-plane. Describes how 'wide' an image ls.
    /// @note For a depth camera implementation the min/max values for each angle are equal in magnitude -- i.e. the field of view is symmetric.
    ///       But this is not necessarily for other implementations of DepthSensorIntrinsics.
    double theta_min, theta_max, phi_min, phi_max ;

    /// @brief Sensor size: (u, v). Number of X-pixels and number if Y-pixels.
    size_t u, v;

    /// @return Field of view in the X-direction in radians.
    /// @note This is always symmetric about the principle axis for a DepthCamera object but necessarily for other implementations of DepthSensorIntrinsics.
    double fov_x() const { return phi_max - phi_min; }

    /// @return Field of view in the Y-direction in radians.
    /// @note This is always symmetric about the principle axis for a DepthCamera object but necessarily for other implementations of DepthSensorIntrinsics.
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
    BaseDepthSensor(const DepthSensorType& sensor_type, const size_t& u, const size_t& v,
                    const double& d_min, const double& d_max,
                    const double& theta_min, const double& theta_max,
                    const double& phi_min, const double& phi_max) :
        type(sensor_type), u(u), v(v),
        d_min(d_min), d_max(d_max),
        theta_min(theta_min), theta_max(theta_max),
        phi_min(phi_min), phi_max(phi_max)
    { }
};


class DepthCamera : public BaseDepthSensor
{
public:
    /// @brief Creates intrinsic matrix for a DepthCamera with the specified parameters.
    /// @param u Sensor dimension, number of of X-pixels.
    /// @param v Sensor dimension, number of of Y-pixels.
    /// @param d_min Minimum depth.
    /// @param d_max Maximum depth.
    /// @param fov_x The field of view (FOV) in the X-direction. The 'width' of the image.
    /// @param fov_y The field of view (FOV) in the X-direction. The 'height' of the image.
    DepthCamera(const size_t& u, const size_t& v, const double& d_min, const double& d_max,
                const double& fov_x, const double& fov_y) :
        BaseDepthSensor(DepthSensorType::DepthCamera, u, v, d_min, d_max,
                        -0.5 * fov_y, 0.5 * fov_y, -0.5 * fov_x, 0.5 * fov_x)
        { }

    /// @brief Generates intrinsics for the Stereo-vision depth camera of an Intel RealSense D455.
    /// @details see spec sheet here:
    ///             https://www.intelrealsense.com/depth-camera-d455/
    ///             https://www.intelrealsense.com/download/20289/?tmstv=1680149335
    static DepthCamera IntelRealSenseD455()
        { return DepthCamera(1280, 800, 0.6, 6, 87 * M_PI / 180, 58 * M_PI / 180); }

    /// @brief Generates intrinsics for the Time-of-Flight depth camera of an Microsoft Azure Kinect.
    /// @param wide_fov If true, default, will return the wide-FOV (WFOV) intrinsics. Else will return the near-FOV (NFOV) intrinsics.
    /// @note WFOV has greater resolution but shorter operating range than the NFOV.
    /// @details see spec sheet here:
    ///             https://learn.microsoft.com/en-us/azure/kinect-dk/hardware-specification#depth-camera-supported-operating-modes
    static DepthCamera AzureKinect(const bool& wide_fov = true) {
        if (wide_fov)
            return DepthCamera(1024, 1024, 0.25, 2.21, 120 * M_PI / 180, 120 * M_PI / 180);
        return DepthCamera(640, 576, 0.3, 3.86, 65 * M_PI / 180, 75 * M_PI / 180);
    }
};


class LaserScanner : public BaseDepthSensor
{
public:
    /// @brief Constructor of the abstract intrinsics class.
    /// @param u Sensor dimension, number of of X-points.
    /// @param v Sensor dimension, number of of Y-points.
    /// @param d_min Minimum depth.
    /// @param d_max Maximum depth.
    /// @param theta_min Minimum FOV angle in the Laser Scanner's YZ-plane.
    /// @param theta_max Maximum FOV angle in the Laser Scanner's YZ-plane.
    /// @param phi_min Minimum FOV angle in the Laser Scanner's XZ-plane.
    /// @param phi_max Maximum FOV angle in the Laser Scanner's XZ-plane.
    LaserScanner(const size_t& u, const size_t& v, const double& d_min, const double& d_max,
                 const double& theta_min, const double& theta_max, const double& phi_min, const double& phi_max) :
        BaseDepthSensor(DepthSensorType::LaserScanner, u, v, d_min, d_max, theta_min, theta_max, phi_min, phi_max)
        { }
};


} // Intrinsics
} // ForgeScan

#endif // FORGESCAN_SENSOR_INTRINSICS_H
