#ifndef FORGESCAN_POLICIES_UNIFORM_H
#define FORGESCAN_POLICIES_UNIFORM_H

#include "ForgeScan/Policies/policy.h"


/**
 * @brief The policies in this file select views by using deterministic algorithms uniform sampling.
 *
 *        UniformSphereOrdered policy uses the Fibonacci sphere algorithm to deterministically calculate approximately
 *        evenly spaced points on a sphere. It requires the number of points to be specified a-priori.
 *        For details on this algorithm and implementation see:
 *            https://stackoverflow.com/questions/9600801/
 *            https://arxiv.org/pdf/0912.4540.pdf
 *
 *        UniformSphereRandom policy will access the UniformSphereOrdered views in a random order.
 */


namespace ForgeScan {
namespace Policies  {


class UniformSphereOrdered : public Policy {
public:
    /// @brief Criteria: Number of views to collect.
    const size_t n_req;

    /// @brief Radius of the camera from the center of the Grid.
    double const radius;

    /// @brief Collects views in a deterministic, uniformly spaced manner.
    /// @details The positioning policy starts by viewing down the Grid's positive Y-axis and with each increment it wraps around
    ///          the Y-axis in a counter-clockwise fashion and descends until it is viewing up the negative Y-axis.
    /// @param grid   Grid for the ongoing reconstruction.
    /// @param sensor Sensor for collecting data from the scene and adding it to the grid.
    /// @param scene  Scene to be reconstructed.
    /// @param n      Number of points to sample.
    /// @param radius Radius of the sphere.
    UniformSphereOrdered(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                         const size_t& n = 15, const double& radius = 2.5) :
        Policy(grid, sensor, scene), n_req(n), radius(radius)
    {
        if (n_req < 1) throw std::invalid_argument("Cannot have less than 1 total view.");
        n_cap = 0;
    }

    /// @brief Checks if the policies stopping criteria are met.
    /// @return True if the criteria are met. False else.
    bool criteriaMet() const override final { return n_cap >= n_req; };

    /// @brief Sets the camera at the next position random position.
    virtual void nextView() override { setNextViewUniformSphere(n_cap); }

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() override { return "UniformSphereOrdered"; }

protected:
    /// @brief Number of images captured so far.
    size_t n_cap;

    /// @brief Sets the position and orientation of the camera based on a uniform spacing algorithm for the number of views requested.
    /// @param n Which uniform point in the set [0, ..., n_req - 1] to set the camera to.
    void setNextViewUniformSphere(const size_t& n) {
        /// Golden angle in radians; see https://en.wikipedia.org/wiki/Golden_angle
        static const double GOLDEN_ANGLE_RADIANS = M_PI * (std::sqrt(5) - 1);

        /// Just less than one; avoids divide by zero errors when just one total view is requested without impacting
        /// the accuracy for the calculation when more total views are requested.
        static const double NEARLY_ONE = 1 - std::numeric_limits<double>::epsilon();

        /// Y walks from 1 to -1
        double y = 1 - (n / ((double)n_req - NEARLY_ONE)) * 2;

        /// Radius of the sphere at that location of y.
        double r_y = std::sqrt(1 - y*y);

        /// Calculate the proper angles. Phi moves from 0 to 180 as y decrements.
        double phi = std::acos(y);
        double theta = GOLDEN_ANGLE_RADIANS * n;

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

    void postRunLoopCall() override final { ++n_cap; };

    virtual void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const override {
        auto file = HighFive::File(fname.string() + ".h5", HighFive::File::ReadWrite);
        auto policy_group = file.createGroup("Policy");
        auto ordered_uniform_group = policy_group.createGroup("UniformSphereOrdered");
        ordered_uniform_group.createAttribute("requested", n_req);
        ordered_uniform_group.createAttribute("captured",  n_cap);
        ordered_uniform_group.createAttribute("radius",    radius);
        ordered_uniform_group.createAttribute("criteria_met", (int)criteriaMet());
    }
};


class UniformSphereRandom : public UniformSphereOrdered {
public:
    /// @brief Uses the same uniform spacing algorithm as UniformSphereOrdered but selects points in a random manner.
    /// @param grid   Grid for the ongoing reconstruction.
    /// @param sensor Sensor for collecting data from the scene and adding it to the grid.
    /// @param scene  Scene to be reconstructed.
    /// @param n      Number of points to sample.
    /// @param radius Radius of the sphere.
    /// @param seed   Seed for the RNG. Negative values will use a random seed. Default is -1.
    UniformSphereRandom(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                        const size_t& n = 15, const double& radius = 2.5, const int& seed = -1) :
        UniformSphereOrdered(grid, sensor, scene, n, radius)
    {
        if (n_req < 1) throw std::invalid_argument("Cannot have less than 1 total view.");
        n_cap = 0;

        /// Used to obtain a seed for the random number engine.
        std::random_device rd;
        this->seed = seed < 0 ? rd() : (unsigned int)seed;
        std::mt19937 gen = std::mt19937(this->seed);

        /// Create the vector of available indices then randomize the order.
        order = std::vector<size_t>(n_req, 0);
        for (size_t i = 0; i < n_req; ++i) order[i] = i;
        std::shuffle(order.begin(), order.end(), gen);
    }

    /// @brief Sets the camera at the next position random position.
    void nextView() override final { setNextViewUniformSphere(order[n_cap]); }

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() override { return "UniformSphereRandom"; }

protected:
    /// @brief Seed for the random number generator.
    unsigned int seed;

    /// @brief Order of point selection.
    std::vector<size_t> order;

    void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const override final {
        auto file = HighFive::File(fname.string() + ".h5", HighFive::File::ReadWrite);
        auto policy_group = file.createGroup("Policy");
        auto ordered_uniform_group = policy_group.createGroup("UniformSphereRandom");
        ordered_uniform_group.createAttribute("requested", n_req);
        ordered_uniform_group.createAttribute("captured",  n_cap);
        ordered_uniform_group.createAttribute("radius",    radius);
        ordered_uniform_group.createAttribute("seed",      seed);
        ordered_uniform_group.createAttribute("criteria_met", (int)criteriaMet());
    }
};


} // namespace Policies
} // namespace ForgeScan

#endif // FORGESCAN_POLICIES_UNIFORM_H
