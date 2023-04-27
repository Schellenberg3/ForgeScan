#ifndef FORGESCAN_SENSOR_INTRINSICS_H
#define FORGESCAN_SENSOR_INTRINSICS_H

#include <cstdint>
#include <cmath>


/// NOTE: I would prefer a more generic BaseImageSensor class to make a CameraSensor then DepthCameraSensor class.
///       But what I have written will do for now.


/// @brief Helper for LaserScannerIntrinsics to properly set the minimum bounds.
/// @param angle_min Desired magnitude of the minimum angle.
/// @param angle_max Magnitude of the maximum angle.
/// @param symmetric If true the maximum angle is returned. If false the user-specified minimum is returned.
/// @return The correct minimum angle based on the symmetry flag a provided values. 
static float setLaserScannerAngleMin(const float& angle_min, const float& angle_max, const bool& symmetric)
{
    return symmetric ? angle_max : angle_min;
}


/// @brief Helper function for DepthCameraIntrinsics to properly set the principle point offset.
/// @param od Provided principle point for the direction.
/// @param n Number of pixels in the direction.
/// @return If od is non-positive the principle point is centered in that direction at 0.5*n. If od is positive
///         then the provided value is returned.
static float setDepthCameraPrinciplePoint(const float& od, const int& n)
{
    return od < 0  ? (float)n/2 : od;
}


/// @brief Generic base class for depth sensor intrinsics.
struct BaseDepthSensorIntrinsics
{
    /// @brief Dimensions of the sensors data: U element in the X-direction and V elements in the Y-direction.
    const uint32_t u, v;

    /// @brief Maximum depth for the sensor.
    const float max_depth;

    /// @brief Creates generic sensor intrinsics for an N by M sensor.
    /// @param max_depth Maximum depth value for the sensor.
    /// @param u Size in the Y-dimension.
    /// @param v Size in the X-dimension.
    /// @param c Number of data channels, including depth (e.g., 1 for only depth or 4 for RGB and depth).
    BaseDepthSensorIntrinsics(float max_depth, uint32_t u, uint32_t v) :
        max_depth(std::fabs(max_depth)),
        u(u),
        v(v)
        { }


    /// @brief Returns the total number of data elements in the sensor.
    /// @return Total number of data elements in the sensor.
    constexpr uint32_t size() const { return u * v; }
};


/// @brief Simplified intrinsics for a laser scanner measurement device.
struct LaserScannerIntrinsics : public BaseDepthSensorIntrinsics
    {
    /// @brief Upper and lower bounds for the scanner's laser in radial distance from the Z-axis (radians).
    ///        Theta describes rotation around the X-axis and phi describes rotation around the Y-axis.
    const float theta_min, theta_max, phi_min, phi_max;


    /// @brief Constructs sensor intrinsics for a laser depth scanner.
    /// @param max_depth Maximum depth value for the laser scanner.
    /// @param u Number of sensed points per-line for the laser scanner (divisions along phi).
    /// @param v Number of scanned lines on the laser scanner (divisions along theta).
    /// @param theta_max Magnitude for the maximum theta value.
    /// @param phi_max   Magnitude for the maximum phi value.
    /// @param theta_min Magnitude for the minimum theta value.
    /// @param phi_min   Magnitude for the minimum phi value.
    /// @param symmetric If true (default), the same magnitude is used for the min/max of theta and phi. If false, then the
    ///                  user-specified values are used.
    /// @note Input signs do not matter for the minimum/maximum for phi or theta. Absolute values are used to ensure that the
    ///       member attributes for maximum are guaranteed to be positive and the minimums are guaranteed to be negative.
    LaserScannerIntrinsics(float max_depth = 100, uint32_t u = 100, uint32_t v = 100,
                           float theta_max = M_PI_4, float phi_max = M_PI_4,
                           float theta_min = M_PI_4, float phi_min = M_PI_4,
                           bool symmetric = true) :
        BaseDepthSensorIntrinsics(max_depth, u, v),
        theta_max(std::fabs(theta_max)),
        phi_max(  std::fabs(phi_max)),
        theta_min(-1 * std::fabs( setLaserScannerAngleMin(theta_min, theta_max, symmetric) ) ),
        phi_min(  -1 * std::fabs( setLaserScannerAngleMin(phi_min, phi_max, symmetric) ) )
        { }
};


/// @brief Simplified intrinsics for a random point laser scanner measurement device.
struct RandomLaserScannerIntrinsics : public LaserScannerIntrinsics
{
    /// @brief Constructs sensor intrinsics for a random point laser depth scanner with a conic shape.
    /// @param max_depth Maximum depth value for the laser scanner.
    /// @param u Number of randomly sensed points for the laser scanner.
    /// @param phi Size of the cone. Half-angle at the conic vertex.
    RandomLaserScannerIntrinsics(float max_depth = 100, uint32_t u = 10000, float phi = M_PI_4) :
        LaserScannerIntrinsics(max_depth, u, 1, M_PI, phi)
        { }
};


struct DepthCameraIntrinsics : public BaseDepthSensorIntrinsics
{
    const float fx, fy;
    const float ox, oy;

    /// @brief Constructs sensor intrinsics for a 
    /// @param max_depth Maximum depth value for the laser scanner.
    /// @param u Number of pixels in the Y-direction.
    /// @param v Number of pixels in the X-direction.
    /// @param fx Focal length in X.
    /// @param fy Focal length in Y (typically equal to X).
    /// @param ox Principle point offset in X.
    /// @param oy Principle point offset in Y.
    /// @note Non-positive values for the principle point offset will default to half the pixels in in that
    ///       direction; essentially centering the principle point offset.
    DepthCameraIntrinsics(float max_depth = 100, uint32_t u = 100,  uint32_t v = 100,
                          float fx = 300, float fy = 300,
                          float ox = -1, float oy = -1) :
        BaseDepthSensorIntrinsics(100, u, v),
        fx(fx), fy(fy),
        ox(setDepthCameraPrinciplePoint(ox, v)),
        oy(setDepthCameraPrinciplePoint(oy, u))
        { }
};


#endif // FORGESCAN_SENSOR_INTRINSICS_H
