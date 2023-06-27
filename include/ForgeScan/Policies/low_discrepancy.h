#ifndef FORGESCAN_POLICIES_LOW_DISCREPANCY_H
#define FORGESCAN_POLICIES_LOW_DISCREPANCY_H

#include <functional>
#include <algorithm>

#include "ForgeScan/Policies/random_sphere.h"


namespace ForgeScan {
namespace Policies  {


class LowDiscrepancy : public RandomSphere
{
public:
    /// @brief A policy based on low-discrepancy sampling of points after a randomly from a spherical surface.
    /// @param grid   Grid for the ongoing reconstruction.
    /// @param sensor Sensor for collecting data from the scene and adding it to the grid.
    /// @param scene  Scene to be reconstructed.
    /// @param n      Number of points to sample.
    /// @param radius Radius of the sphere.
    /// @param seed   Seed for the RNG. Negative values will use a random seed. Default is -1.
    LowDiscrepancy(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                   const int& n = 15, const double& radius = 2.5, const int& seed = -1) :
        RandomSphere(grid, sensor, scene, n, radius, seed)
        { }

    /// @brief Sets the next position using the low-discrepancy policy. The policy begins with three orthogonal views.
    ///        In the case where a unique low-discrepancy view cannot be found, a random position is chosen.
    virtual void nextPosition() override {
        if (n_cap < 3) {
            setNextPositionOrthogonal();
        } else {
            setNextPositionLowDiscrepancy();
        }
    }

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() { return "LowDiscrepancy"; }

protected:
    /// @brief Sets the next position as the orthogonal view in X, Y, then Z as the policy is run.
    /// @note  If the current number of captures is greater than 3 then this runs the low-discrepancy search.
    void setNextPositionOrthogonal() {
        if (n_cap > 3) setNextPositionLowDiscrepancy();

        /// Center of the Grid, relative to the Grid's frame.
        const point grid_center = 0.5 * grid->properties.dimensions;

        /// Start position as all zeros, relative to the Grid's frame.
        Eigen::Vector3d unit(0,0,0);

        /// A bit hacky, but n_cap is < 0 and < 3, so as it is incremented we will select the X, Y, then
        /// Z index as the orthogonal direction to view from. We immediately set this to the view radius.
        unit[n_cap] = radius;

        /// Position of the sensor, relative to the Grid's frame.
        point position =  unit + grid_center;

        grid->toWorldFromThis(position); // Transform to the world reference frame.

        sensor->extr.setIdentity();
        sensor->translate(position);     // Place the sensor in the new position in the World reference frame.
        orientSensorToGridCenter();
    }

    /// @brief Sets the next position as far from existing positions as possible. If a unique position cannot
    ///        be identified then a random position is selected.
    void setNextPositionLowDiscrepancy() {
        /// Centroid and position are both relative to the Grid's reference frame.
        point centroid(0, 0, 0), position(0, 0, 0);
        for (const auto& rec : sensor_record.record) { centroid += rec.pose.translation(); }
        centroid /= static_cast<double>(sensor_record.record.size());

        if ( seachLowDiscrepancy(centroid, position) ) {
            grid->toWorldFromThis(position); // Transform to the world reference frame.
            sensor->extr.setIdentity();
            sensor->translate(position);     // Place the sensor in the new position in the World reference frame.
            orientSensorToGridCenter();
        } else {
            setNextPositionRandom();
        }
    }

    /// @brief Performs a low-discrepancy search for the next location on our imaging sphere. To resolve ambiguity
    ///        it may remove random points from the centroid calculation.
    /// @param centroid  Centroid of the existing cameras.
    /// @param out       Output point; next position for the sensor.
    /// @param max_depth Max recursive search. Ends search and returns false when depth equals this
    ///                  value. Defaults is 3. If greater than the one less than the number of sensors in
    ///                  the sensor_record then it is set to that value instead.
    /// @return True if there is a valid, unique surface point for the centroid.
    bool seachLowDiscrepancy(const Vector3d& centroid, point& out, size_t max_depth = 3) {
        /// Center of the grid, relative to the Grid's reference frame.
        const point grid_center = 0.5 * grid->properties.dimensions;

        /// Local, modifiable copy of the input centroid,
        point local_centroid = centroid;

        /// Subtract one from the number of records. We do not want to remove the last point we added
        /// as then we would, likely, just select it again.
        const size_t n_record_ajd = sensor_record.record.size() - 1;

        /// Randomized order of record indicies to remove if a unique point is not found.
        std::vector<size_t> randomized_idx(n_record_ajd, 0);
        for (int i = 0; i < randomized_idx.size(); ++i) randomized_idx[i] = i;
        std::shuffle(randomized_idx.begin(), randomized_idx.end(), gen);

        /// Adjust max depth in case it is greater than the number of records we have available.
        max_depth = std::min(max_depth, n_record_ajd);

        /// Tracks the recursive depth.
        size_t depth = 0;

        /// Implements our low-discrepancy search with recursion for ambiguous cases.
        std::function<bool()> recursive_search;

        recursive_search = [&](){
            /// After several attempts we simply return false to default to a random strategy.
            if (depth >= max_depth) return false;

            /// Ray from the center of the Grid to the centroid (both in the Grid's reference frame).
            const Eigen::Vector3d ray = local_centroid - grid_center;
            const double len = ray.norm();

            /// Call recursion if our centroid is too close to the center, indicating that there is some symmetry
            /// that this simple method cannot disambiguate. (Or, the likely impossible case, that our centroid is
            /// outside of the circle.)
            if (len > radius || len < 1E-8) {
                /// Recover the sum of points from the centroid.
                local_centroid *= static_cast<double>( sensor_record.record.size() );

                /// Remove the contribution of the randomly selected point.
                local_centroid -= sensor_record.record[randomized_idx[depth]].pose.translation();

                /// Increment depth counter for the next call. We do this now to use it in updating the local centroid.
                ++depth;

                /// Centroid is now adjusted to have the selected point removed.
                local_centroid /= static_cast<double>( (sensor_record.record.size() - depth) );

                /// Call ourselves once more...
                return recursive_search();
            }

            /// If the ray ends within the sphere, then we scale the ray and march in the OPPOSITE
            /// direction. Since the centroid is close to the sampled points we want to move away.
            /// The variable out is relative to the Grid reference frame NOT the center.
            out = grid_center - ray * (radius / len);
            return true;
        };

        /// Call our recursive search which will modify both centroid and out, returning true if they are valid.
        return recursive_search();
    }

    virtual void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const override {
        auto file = HighFive::File(fname.string() + ".h5", HighFive::File::ReadWrite);
        auto policy_group = file.createGroup("Policy");
        auto random_sphere = policy_group.createGroup("LowDiscrepancy");
        random_sphere.createAttribute("requested",    n_req);
        random_sphere.createAttribute("captured",     n_cap);
        random_sphere.createAttribute("seed",         seed);
        random_sphere.createAttribute("radius",       radius);
        random_sphere.createAttribute("criteria_met", (int)criteriaMet());
    }
};

class LowDiscrepancyRandomInit : public LowDiscrepancy
{
public:
    /// @brief Number of initial random views.
    const int n_random;

    /// @brief A policy based on low-discrepancy sampling of points after a randomly from a spherical surface.
    /// @param grid   Grid for the ongoing reconstruction.
    /// @param sensor Sensor for collecting data from the scene and adding it to the grid.
    /// @param scene  Scene to be reconstructed.
    /// @param n      Number of points to sample.
    /// @param radius Radius of the sphere.
    /// @param i      Number of random points to initialize with. Default is 3.
    /// @param seed   Seed for the RNG. Negative values will use a random seed. Default is -1.
    LowDiscrepancyRandomInit(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene,
                             const int& n = 15, const double& radius = 2.5, const int& i = 3, const int& seed = -1) :
        LowDiscrepancy(grid, sensor, scene, n, radius, seed), n_random(i < 0 ? 3 : i)
        { }

    /// @brief Sets the next position using the low-discrepancy policy.
    void nextPosition() override final {
        if (n_cap < n_random) {
            return setNextPositionRandom();
        } else {
            setNextPositionLowDiscrepancy();
        }
    }

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() { return "LowDiscrepancyRandomInit"; }

protected:
    void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const override final {
        auto file = HighFive::File(fname.string() + ".h5", HighFive::File::ReadWrite);
        auto policy_group = file.createGroup("Policy");
        auto random_sphere = policy_group.createGroup("LowDiscrepancyRandomInit");
        random_sphere.createAttribute("requested",    n_req);
        random_sphere.createAttribute("captured",     n_cap);
        random_sphere.createAttribute("random_init",  n_random);
        random_sphere.createAttribute("seed",         seed);
        random_sphere.createAttribute("radius",       radius);
        random_sphere.createAttribute("criteria_met", (int)criteriaMet());
    }
};


} // namespace Policies
} // namespace ForgeScan


#endif // FORGESCAN_POLICIES_LOW_DISCREPANCY_H
