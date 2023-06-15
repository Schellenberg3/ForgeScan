#ifndef FORGESCAN_POLICIES_ORDERED_UNIFORM_H
#define FORGESCAN_POLICIES_ORDERED_UNIFORM_H

#include "ForgeScan/Policies/policy.h"


namespace ForgeScan {
namespace Policies  {


class OrderedUniform : public Policy {
public:
    /// @brief Criteria: Number of views to collect.
    const int n_req;

    /// @brief Radius of the camera from the center of the Grid.
    double const radius;

    OrderedUniform(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                   const int& n = 15, const double& radius = 2.5) :
        Policy(grid, sensor, scene), n_req(n), radius(radius)
    {
        if (n_req < 1) throw std::invalid_argument("Cannot have less than 1 total view.");
        n_cap = 0;
    }

    /// @brief Checks if the policies stopping criteria are met.
    /// @return True if the criteria are met. False else.
    bool criteriaMet() override final { return n_cap >= n_req; };

    /// @brief Sets the camera at the next position random position.
    void nextPosition() override final {
        static const double GOLDEN_ANGLE_RADIANS = M_PI * (std::sqrt(5) - 1);

        /// Cast to double for later.
        double n_req_d = (double)n_req;

        /// Slight bump to prevent divide by zero errors.
        if (n_req == 1) n_req_d += 0.000000001;

        /// Y walks from 1 to -1
        double y = 1 - (n_cap / (n_req_d - 1)) * 2;

        /// Radius of the sphere at that location of y.
        double r_y = std::sqrt(1 - y*y);

        /// Calculate the proper angles. Phi moves from 0 to 180 as y decrements.
        double phi = std::acos(y);
        double theta = GOLDEN_ANGLE_RADIANS * n_cap;

        /// Solve the last two distances.
        double x = std::cos(theta) * r_y;
        double z = std::sin(theta) * r_y;

        point position(x, y, z);
        position *= radius;
        position += grid->getCenter();

        sensor->extr.setIdentity();
        sensor->translate(position);

        orientSensorToGridCenter();
    }

private:
    /// @brief Number of images captured so far.
    int n_cap;

    void postRunLoopCall() override final { ++n_cap; };
};


} // namespace Policies
} // namespace ForgeScan

#endif // FORGESCAN_POLICIES_ORDERED_UNIFORM_H
