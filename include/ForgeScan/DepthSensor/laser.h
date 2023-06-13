#ifndef FORGESCAN_DEPTH_SENSOR_LASER_H
#define FORGESCAN_DEPTH_SENSOR_LASER_H

#include "ForgeScan/DepthSensor/sensor.h"
#include "ForgeScan/DepthSensor/intrinsics.h"

namespace ForgeScan   {
namespace DepthSensor {

namespace Intrinsics  {


class Laser : public Intrinsics
{
public:
    /// @brief Creates intrinsic for a Laser Scanner type DepthSensor with the specified parameters.
    /// @param u Sensor dimension, number of of X-points.
    /// @param v Sensor dimension, number of of Y-points.
    /// @param d_min Minimum depth.
    /// @param d_max Maximum depth.
    /// @param theta_min Minimum FOV angle in the Laser Scanner's YZ-plane.
    /// @param theta_max Maximum FOV angle in the Laser Scanner's YZ-plane.
    /// @param phi_min Minimum FOV angle in the Laser Scanner's XZ-plane.
    /// @param phi_max Maximum FOV angle in the Laser Scanner's XZ-plane.
    Laser(const size_t& u, const size_t& v, const double& d_min, const double& d_max,
                           const double& theta_min, const double& theta_max, const double& phi_min, const double& phi_max) :
        Intrinsics(Type::Laser, u, v, d_min, d_max, theta_min, theta_max, phi_min, phi_max)
        { }
};



} // Intrinsics


class Laser : public Sensor
{
public:
    Laser(const Intrinsics::Laser& intr) :
        Sensor(intr)
        { setup(); }

    Laser(const Intrinsics::Laser& intr, const extrinsic& extr) :
        Sensor(intr, extr)
        { setup(); }

    Laser(const Intrinsics::Laser& intr, const translation& position) :
        Sensor(intr, position)
        { setup(); }

    Laser(const Intrinsics::Laser& intr, const rotation& orientation) :
        Sensor(intr, orientation)
        { setup(); }

    void resetDepth() override final { resetDepthLaser(); }

private:
    void setup() { resetDepthLaser(); }
};


} // DepthSensor
} // ForgeScan

#endif // FORGESCAN_DEPTH_SENSOR_LASER_H
