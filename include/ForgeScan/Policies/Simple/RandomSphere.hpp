#ifndef FORGE_SCAN_POLICIES_SIMPLE_RANDOM_SPHERE_POLICY_HPP
#define FORGE_SCAN_POLICIES_SIMPLE_RANDOM_SPHERE_POLICY_HPP

#include <limits>

#include "ForgeScan/Policies/Policy.hpp"

#include "ForgeScan/Common/VectorMath.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Random.hpp"


namespace forge_scan {
namespace policies {


class RandomSphere : public Policy
{
public:
    /// @brief Creates a Random Sphere Policy.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param parser Arg Parser with arguments to construct an Random Sphere Policy from.
    /// @return Shared pointer to a Random Sphere Policy.
    static std::shared_ptr<RandomSphere> create(const std::shared_ptr<const data::Reconstruction>& reconstruction,
                                                const utilities::ArgParser& parser)
    {
        return std::shared_ptr<RandomSphere>(new RandomSphere(reconstruction,
                                                              parser.getCmdOption<size_t>("--n-views", 10),
                                                              parser.getCmdOption<float>("--r-min", 2.5),
                                                              parser.getCmdOption<float>("--r-max", 2.5),
                                                              parser.getCmdOption<float>("--seed", -1)));
    }


private:
    /// @brief Private constructor to enforce usage of shared pointer.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param n_views Number of accepted views this Policy is required to generate before it
    ///                considers the Reconstruction complete.
    /// @param r_min Minimum radius for camera poses.
    /// @param r_max Maximum radius for camera poses.
    /// @param seed Seed for the random generator the Policy uses. Default -1 for a random seed.
    explicit RandomSphere(const std::shared_ptr<const data::Reconstruction>& reconstruction,
                          const size_t& n_views,
                          const float& r_min,
                          const float& r_max,
                          const float& seed)
        : Policy(reconstruction),
          n_view_requested(n_views),
          r_min(std::min(std::abs(r_min), std::abs(r_max))),
          r_max(std::max(std::abs(r_min), std::abs(r_max))),
          seed(seed),
          sample(this->seed)
    {
        // This Policy can generate on startup.
        this->generate();
    }



    // ***************************************************************************************** //
    // *                           PRIVATE VIRTUAL METHOD OVERRIDES                            * //
    // ***************************************************************************************** //


    virtual const std::string& getTypeName() const override final
    {
        const static std::string name("RandomSphere");
        return name;
    }


    virtual void generate() override final
    {
        float theta, phi, r;

        const Point grid_center = this->reconstruction->grid_properties->getCenter();
        Extrinsic extr = Extrinsic::Identity();

        this->sample.sphere(theta, phi, true);
        r = this->sample.uniform(this->r_min, this->r_max);

        extr.translation() = vector_math::spherical_to_cartesian(r, theta, phi);

        // At this point the pose is relative to the center of the scan area.
        // We need to shift it to be relative to the scan lower bound.
        extr.translation() += grid_center;

        // Now we point this pose towards the Grid's center point.
        extr.rotate(vector_math::get_rotation_to_orient_z_axis(extr, grid_center));

        // For this policy, clear any views before adding the newly generated one.
        this->views.clear();
        this->views.push_back(extr);
    }


    bool isComplete() const override final
    {
        return this->numAccepted() >= this->n_view_requested;
    }


    void save(H5Easy::File& file, HighFive::Group& g_policy) const override final
    {
        auto g_rand_sph = g_policy.createGroup(this->getTypeName());
        g_rand_sph.createAttribute("r_min", this->r_min);
        g_rand_sph.createAttribute("r_max", this->r_max);
        g_rand_sph.createAttribute("n_view_requested", this->n_view_requested);
        g_rand_sph.createAttribute("seed", this->seed);
        g_rand_sph.createAttribute("completed", static_cast<uint8_t>(this->isComplete()));
        Policy::saveRejectedViews(file, this->getTypeName());
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief Number of views requested by the user. Once this value is exceeded the Policy is considered complete.
    size_t n_view_requested;

    /// @brief Minimum and maximum radius for sampling positions.
    const float r_min, r_max;

    /// Seed for the random sample. (-1 indicates a random seed is used).
    float seed;

    /// @brief Random sampler utility.
    utilities::RandomSampler<float> sample;
};


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_SIMPLE_RANDOM_SPHERE_POLICY_HPP
