#ifndef FORGESCAN_POLICIES_LOW_DISCREPANCY_H
#define FORGESCAN_POLICIES_LOW_DISCREPANCY_H

#include "ForgeScan/Policies/random.h"


namespace ForgeScan {
namespace Policies  {


class LowDiscrepancySphere : public RandomSphere {
public:
    /// @brief A policy based on randomly sampling views from a spherical surface.
    /// @param grid   Grid for the ongoing reconstruction.
    /// @param sensor Sensor for collecting data from the scene and adding it to the grid.
    /// @param scene  Scene to be reconstructed.
    /// @param n      Number of views to sample.
    /// @param radius Radius of the sphere.
    /// @param min_arc_len  Required distance between sampled points. Default is 0.1.
    /// @param max_attempts Maximum tries at searching for a low-discrepancy point before settling for a random point.
    ///                     Default is 10 attempts.
    /// @param seed   Seed for the RNG. Negative values will use a random seed. Default is -1.
    LowDiscrepancySphere(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                         const size_t& n = 15, const double& radius = 2.5, const double& arc_len = 0.1,
                         const int& seed = -1, const size_t& max_attempts = 10) :
        RandomSphere(grid, sensor, scene, n, radius, seed), arc_len(std::abs(arc_len)), max_attempts(max_attempts)
        {
            theta_v.reserve(n);
            phi_v.reserve(n);
            broke_discrepancy = 0;
        }

    /// @brief Sets the camera at the next position random position.
    virtual void nextView() override {
        double theta = 0, phi = 0;
        size_t attempts = 0;

        /// Search for a new theta/phi pair for a view.
        do {
            getRandomThetaPhiPair(theta, phi);

            if (++attempts >= max_attempts) {
                ++broke_discrepancy;
                break;
            }
        } while (!checkDiscrepancy(theta, phi));
        
        /// Record the new point and set the sensor view to the appropriate position.
        theta_v.push_back(theta);
        phi_v.push_back(phi);
        setNextViewSpherical(theta, phi);
    }

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() override { return "LowDiscrepancySphere"; }

protected:
    /// @brief Required distance between sampled points.
    const double arc_len;

    /// @brief Maximum attempts to randomly select a low-discrepancy point before defaulting to a random point.
    const size_t max_attempts;

    /// @brief Count of how many points broke the low-discrepancy request to meet the requested number of views.
    size_t broke_discrepancy;

    /// @brief Record of the theta and phi values for views added to the spherical surface.
    std::vector<double> theta_v, phi_v;

    /// @brief Checks if the provided point than the minimum arc-length from all existing points on the sphere.
    /// @param theta Theta of the test point.
    /// @param phi   Phi of the test point.
    /// @returns True if the test point is further than the minimum arc length from all existing points.
    /// @details The arc-length algorithm used comes from:
    ///              https://math.stackexchange.com/a/231225
    bool checkDiscrepancy(const double& theta, const double& phi) {
        const double sin_phi = std::sin(phi), cos_phi = std::cos(phi);
        double len = 0;
        for(size_t i = 0; i < n_cap; ++i) {
            len = radius * std::acos( (std::cos(phi_v[i]) * cos_phi) + (std::sin(phi_v[i]) * sin_phi * std::cos(theta_v[i] - theta)) );
            if (len < arc_len) return false;
        }
        return true;
    }

    virtual void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const override {
        auto file = HighFive::File(fname.string() + ".h5", HighFive::File::ReadWrite);
        auto policy_group = file.createGroup("Policy");
        auto random_sphere = policy_group.createGroup("LowDiscrepancySphere");
        random_sphere.createAttribute("requested",    n_req);
        random_sphere.createAttribute("captured",     n_cap);
        random_sphere.createAttribute("seed",         seed);
        random_sphere.createAttribute("radius",       radius);
        random_sphere.createAttribute("arc_len",      arc_len);
        random_sphere.createAttribute("broke_discrepancy", broke_discrepancy);
        random_sphere.createAttribute("criteria_met",      (int)criteriaMet());
    }
};


} // namespace Policies
} // namespace ForgeScan


#endif // FORGESCAN_POLICIES_LOW_DISCREPANCY_H
