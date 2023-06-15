#ifndef FORGESCAN_DEPTH_SENSOR_CAMERA_H
#define FORGESCAN_DEPTH_SENSOR_CAMERA_H

#include "ForgeScan/DepthSensor/sensor.h"
#include "ForgeScan/DepthSensor/intrinsics.h"


namespace ForgeScan   {
namespace DepthSensor {
namespace Intrinsics  {


class Camera : public Intrinsics
{
public:
    /// @brief Creates intrinsic for a Depth Camera type DepthSensor with the specified parameters.
    /// @param u Sensor dimension, number of of X-pixels.
    /// @param v Sensor dimension, number of of Y-pixels.
    /// @param d_min Minimum depth.
    /// @param d_max Maximum depth.
    /// @param fov_x The field of view (FOV) in the X-direction. The 'width' of the image.
    /// @param fov_y The field of view (FOV) in the X-direction. The 'height' of the image.
    Camera(const size_t& u, const size_t& v, const double& d_min, const double& d_max,
                const double& fov_x, const double& fov_y) :
        Intrinsics(Type::Camera, u, v, d_min, d_max, -0.5 * fov_y, 0.5 * fov_y, -0.5 * fov_x, 0.5 * fov_x)
        { }

    /// @brief Generates intrinsics for the Stereo-vision depth camera of an Intel RealSense D455.
    /// @details see spec sheet here:
    ///             https://www.intelrealsense.com/depth-camera-d455/
    ///             https://www.intelrealsense.com/download/20289/?tmstv=1680149335
    static Camera IntelRealSenseD455()
        { return Camera(1280, 800, 0.6, 6, 87 * M_PI / 180, 58 * M_PI / 180); }

    /// @brief Generates intrinsics for the Time-of-Flight depth camera of an Microsoft Azure Kinect.
    /// @param wide_fov If true, default, will return the wide-FOV (WFOV) intrinsics. Else will return the near-FOV (NFOV) intrinsics.
    /// @note WFOV has greater resolution but shorter operating range than the NFOV.
    /// @details see spec sheet here:
    ///             https://learn.microsoft.com/en-us/azure/kinect-dk/hardware-specification#depth-camera-supported-operating-modes
    static Camera AzureKinect(const bool& wide_fov = true) {
        if (wide_fov)
            return Camera(1024, 1024, 0.25, 2.21, 120 * M_PI / 180, 120 * M_PI / 180);
        return Camera(640, 576, 0.3, 3.86, 65 * M_PI / 180, 75 * M_PI / 180);
    }
};


} // namespace Intrinsics


class Camera : public Sensor
{
public:
    Camera(const Intrinsics::Camera& intr) :
        Sensor(intr)
        { setup(); }

    Camera(const Intrinsics::Camera& intr, const extrinsic& extr) :
        Sensor(intr, extr)
        { setup(); }

    Camera(const Intrinsics::Camera& intr, const translation& position) :
        Sensor(intr, position)
        { setup(); }

    Camera(const Intrinsics::Camera& intr, const rotation& orientation) :
        Sensor(intr, orientation)
        { setup(); }

    void resetDepth() override final { resetDepthCamera(); }

private:
    void setup() { resetDepthCamera(); }
};


} // namespace DepthSensor
} // namespace ForgeScan

#endif // FORGESCAN_DEPTH_SENSOR_CAMERA_H
