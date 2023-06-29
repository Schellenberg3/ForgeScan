#ifndef FORGESCAN_POLICIES_RANDOM_H
#define FORGESCAN_POLICIES_RANDOM_H

#include "ForgeScan/Policies/policy.h"


/**
 * @brief The policies in this file select the next view based on purely stochastic methods.
 *
 *        RandomSphere selects random views constraining to a spherical surface around the Grid's center
 */


namespace ForgeScan {
namespace Policies  {


class RandomSphere : public Policy {
public:
    /// @brief Criteria: Number of views to collect.
    const size_t n_req;

    /// @brief Radius of the camera from the center of the Grid.
    double const radius;

    /// @brief A policy based on randomly sampling views from a spherical surface.
    /// @param grid   Grid for the ongoing reconstruction.
    /// @param sensor Sensor for collecting data from the scene and adding it to the grid.
    /// @param scene  Scene to be reconstructed.
    /// @param n      Number of views to sample.
    /// @param radius Radius of the sphere.
    /// @param seed   Seed for the RNG. Negative values will use a random seed. Default is -1.
    RandomSphere(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                 const size_t& n = 15, const double& radius = 2.5, const int& seed = -1) :
        Policy(grid, sensor, scene), n_req(n), radius(radius)
    {
        /// Used to obtain a seed for the random number engine.
        std::random_device rd;
        this->seed = seed < 0 ? rd() : (unsigned int)seed;
        n_cap = 0;
        uniform_dist = std::uniform_real_distribution<double>(0.0, 1.0);
        gen = std::mt19937(this->seed);
    }

    /// @brief Checks if the policies stopping criteria are met.
    /// @return True if the criteria are met. False else.
    bool criteriaMet() const override final { return n_cap >= n_req; };

    /// @brief Sets the camera at the next position random position.
    virtual void nextView() override { setNextViewRandomSphere(); }

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() { return "RandomSphere"; }

protected:
    /// @brief Seed for the random number generator.
    unsigned int seed;

    /// @brief Number of images captured so far.
    size_t n_cap;

    /// @brief Random number engine for performing sampling on the uniform real distribution.
    std::mt19937 gen;

    /// @brief Uniform distribution over [0, 1).
    std::uniform_real_distribution<double> uniform_dist;

    /// @brief Selects a random view on the sphere to move the camera to and orients it at the center of the grid.
    void setNextViewRandomSphere() {
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

    void postRunLoopCall() override final { ++n_cap; };

    virtual void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const override {
        auto file = HighFive::File(fname.string() + ".h5", HighFive::File::ReadWrite);
        auto policy_group = file.createGroup("Policy");
        auto random_sphere = policy_group.createGroup("RandomSphere");
        random_sphere.createAttribute("requested",    n_req);
        random_sphere.createAttribute("captured",     n_cap);
        random_sphere.createAttribute("seed",         seed);
        random_sphere.createAttribute("radius",       radius);
        random_sphere.createAttribute("criteria_met", (int)criteriaMet());
    }
};


} // namespace Policies
} // namespace ForgeScan

#endif // FORGESCAN_POLICIES_RANDOM_H
