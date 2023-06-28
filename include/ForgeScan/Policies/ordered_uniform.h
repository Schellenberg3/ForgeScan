#ifndef FORGESCAN_POLICIES_ORDERED_UNIFORM_H
#define FORGESCAN_POLICIES_ORDERED_UNIFORM_H

#include <highfive/H5File.hpp>

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
    bool criteriaMet() const override final { return n_cap >= n_req; };

    /// @brief Sets the camera at the next position random position.
    void nextPosition() override final {
        /// Golden angle in radians; see https://en.wikipedia.org/wiki/Golden_angle
        static const double GOLDEN_ANGLE_RADIANS = M_PI * (std::sqrt(5) - 1);

        /// Just less than one; avoids divide by zero errors when just one total view is requested without impacting
        /// the accuracy for the calculation when more total views are requested.
        static const double NEARLY_ONE = 1 - std::numeric_limits<double>::epsilon();

        /// Y walks from 1 to -1
        double y = 1 - (n_cap / ((double)n_req - NEARLY_ONE)) * 2;

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

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() { return "OrderedUniform"; }

private:
    /// @brief Number of images captured so far.
    int n_cap;

    void postRunLoopCall() override final { ++n_cap; };

    void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const override final {
        auto file = HighFive::File(fname.string() + ".h5", HighFive::File::ReadWrite);
        auto policy_group = file.createGroup("Policy");
        auto ordered_uniform_group = policy_group.createGroup("OrderedUniform");
        ordered_uniform_group.createAttribute("requested", n_req);
        ordered_uniform_group.createAttribute("captured",  n_cap);
        ordered_uniform_group.createAttribute("radius",    radius);
        ordered_uniform_group.createAttribute("criteria_met", (int)criteriaMet());
    }
};


} // namespace Policies
} // namespace ForgeScan

#endif // FORGESCAN_POLICIES_ORDERED_UNIFORM_H