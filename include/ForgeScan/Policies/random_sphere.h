#ifndef FORGESCAN_POLICIES_RANDOM_SPHERE_H
#define FORGESCAN_POLICIES_RANDOM_SPHERE_H

#include "ForgeScan/Policies/policy.h"


namespace ForgeScan {
namespace Policies  {


class RandomSphere : public Policy {
public:
    /// @brief Criteria: Number of views to collect.
    const int n_req;

    /// @brief Radius of the camera from the center of the Grid.
    double const radius;

    RandomSphere(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                 const int& n = 15, const double& radius = 2.5, const int& seed = 0) :
        Policy(grid, sensor, scene), n_req(n), radius(radius)
    {
        n_cap = 0;
        uniform_dist = std::uniform_real_distribution<double>(0.0, 1.0);
        gen = std::mt19937(seed);
    }

    /// @brief Checks if the policies stopping criteria are met.
    /// @return True if the criteria are met. False else.
    bool criteriaMet() override final { return n_cap >= n_req; };

    /// @brief Sets the camera at the next position random position.
    void nextPosition() override final {
        double theta = 2 * M_PI * uniform_dist(gen);        /// 0 - 360 degrees in theta  (angle around the positive X-axis)
        double phi = std::acos(1 - 2 * uniform_dist(gen));  /// 0 - 180 degrees in phi    (angle from positive Z-axis)
        if (uniform_dist(gen) < 0.5) phi *= -1;             /// -180 - 180 degrees in phi (important for covering all camera orientations)

        point position(std::sin(theta) * std::sin(phi), std::cos(theta) * std::sin(phi), std::cos(phi));
        position *= radius;
        position += grid->getCenter();

        sensor->extr.setIdentity();
        sensor->translate(position);

        orientSensorToGridCenter();
    }

private:
    /// @brief Number of images captured so far.
    int n_cap;

    /// @brief Random number engine for performing sampling on the uniform real distribution.
    std::mt19937 gen;

    /// @brief Uniform distribution over [0, 1).
    std::uniform_real_distribution<double> uniform_dist;

    void postRunLoopCall() override final { ++n_cap; };
};


} // namespace Policies
} // namespace ForgeScan

#endif // FORGESCAN_POLICIES_RANDOM_SPHERE_H
