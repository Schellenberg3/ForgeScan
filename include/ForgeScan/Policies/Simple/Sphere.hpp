#ifndef FORGE_SCAN_POLICIES_SIMPLE_SPHERE_POLICY_HPP
#define FORGE_SCAN_POLICIES_SIMPLE_SPHERE_POLICY_HPP

#include <algorithm> 
#include <limits>
#include <functional>

#include "ForgeScan/Policies/Policy.hpp"

#include "ForgeScan/Common/VectorMath.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Random.hpp"


namespace forge_scan {
namespace policies {


class Sphere : public Policy
{
public:
    /// @brief Creates a Sphere Policy.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param parser Arg Parser with arguments to construct an Sphere Policy from.
    /// @return Shared pointer to a Sphere Policy.
    static std::shared_ptr<Sphere> create(const std::shared_ptr<const data::Reconstruction>& reconstruction,
                                                const utilities::ArgParser& parser)
    {
        return std::shared_ptr<Sphere>(new Sphere(reconstruction,
                                                  parser.getCmdOption<size_t>("--n-views",   10),
                                                  parser.cmdOptionExists("--uniform"),
                                                  parser.cmdOptionExists("--unordered"),
                                                  parser.getCmdOption<float>("--radius",     2.5),
                                                  parser.getCmdOption<float>("--radius-max", 2.5),
                                                  parser.getCmdOption<float>("--seed",       -1)));
    }


protected:
    /// @brief Protected constructor to enforce usage of shared pointer.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param n_views Number of accepted views this Policy is required to generate before it
    ///                considers the Reconstruction complete.
    /// @param uniform Flag, if true then the this Policy starts be generating `n_views` worth of
    ///                ordered, uniformly sampled views. If false, or one more than `v_views` have
    ///                been generated in the uniform manner, the policy samples the spherical
    ///                surface randomly.
    /// @param unordered  If this is true and uniform is true, then the policy will return uniformly
    ///                   sampled points but visits them in an unordered manner. A Sphere policy with
    ///                   `uniform` and `unordered` sampling flags is pseudo low-discrepancy.
    /// @param radius     Minimum radius for camera poses.
    /// @param radius_max Maximum radius for camera poses, used for randomly sampled views only.
    /// @param seed Seed for the random generator the Policy uses. Default -1 for a random seed.
    explicit Sphere(const std::shared_ptr<const data::Reconstruction>& reconstruction,
                    const size_t& n_views,
                    const bool& uniform,
                    const bool& unordered,
                    const float& radius,
                    const float& radius_max,
                    const float& seed)
        : Policy(reconstruction),
          n_view_requested(n_views),
          start_uniform(uniform),
          unordered(unordered),
          radius(std::min(std::abs(radius), std::abs(radius_max))),
          radius_max(std::max(std::abs(radius), std::abs(radius_max))),
          seed(seed),
          sample(this->seed)
    {
        if (this->start_uniform)
        {
            this->call_on_generate = std::bind(&Sphere::generateUniform, this, std::placeholders::_1);
            this->view_order = std::vector<size_t>(this->n_view_requested);
            std::iota(this->view_order.begin(), this->view_order.end(), 0);
            if (this->unordered)
            {
                std::shuffle(this->view_order.begin(), this->view_order.end(), this->sample.gen);
            }
        }
        else
        {
            this->call_on_generate = std::bind(&Sphere::generateRandom, this, std::placeholders::_1);
        }

        // This Policy can generate on startup.
        this->generate();
    }


    /// @brief Samples a random position on the surface of a sphere.
    /// @param dest Extrinsic matrix in which the position is stored.
    void generateRandom(Extrinsic& dest)
    {
        float theta, phi, r;

        this->sample.sphere(theta, phi, true);
        r = this->sample.uniform(this->radius, this->radius_max);

        dest.translation() = vector_math::spherical_to_cartesian(r, theta, phi);
    }


    /// @brief Samples uniform, ordered positions on the surface of a sphere.
    /// @param dest Extrinsic matrix in which the position is stored.
    void generateUniform(Extrinsic& dest)
    {
        // Golden angle in radians. See: https://en.wikipedia.org/wiki/Golden_angle
        static const float golden_angle_radians = M_PI * (std::sqrt(5) - 1);

        // Avoids division by zero errors is n_view_requested is 1.
        static const float nearly_one = 1 - std::numeric_limits<float>::epsilon();

        const size_t view_number = this->numAccepted() + this->numRejected();
        if (this->n_view_requested <= view_number)
        {
            // One we have reached the last view we may generate we switch to random sampling.
            this->call_on_generate = std::bind(&Sphere::generateRandom, this, std::placeholders::_1);
            this->generateRandom(dest);
            return;
        }

        float y = 1 - (this->view_order[view_number] / ((float)n_view_requested - nearly_one)) * 2;
        float r_y = std::sqrt(1 - y*y);

        float theta = golden_angle_radians * this->view_order[view_number];

        float x = std::cos(theta) * r_y;
        float z = std::sin(theta) * r_y;

        Point position(x, y, z);
        position *= this->radius;

        dest.translation() = position;
    }


    // ***************************************************************************************** //
    // *                           PRIVATE VIRTUAL METHOD OVERRIDES                            * //
    // ***************************************************************************************** //


    virtual const std::string& getTypeName() const override final
    {
        const static std::string name("Sphere");
        return name;
    }


    virtual void generate() override final
    {
        const Point grid_center = this->reconstruction->grid_properties->getCenter();
     
        Extrinsic extr = Extrinsic::Identity();
        this->call_on_generate(extr);
        
        extr.translation() += grid_center;
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
        g_rand_sph.createAttribute("radius",     this->radius);
        g_rand_sph.createAttribute("radius_max", this->radius_max);
        g_rand_sph.createAttribute("n_view_requested", this->n_view_requested);
        g_rand_sph.createAttribute("start_uniform",    static_cast<uint8_t>(this->start_uniform));
        g_rand_sph.createAttribute("seed", this->seed);
        g_rand_sph.createAttribute("completed", static_cast<uint8_t>(this->isComplete()));
        Policy::saveRejectedViews(file, this->getTypeName());
        Policy::saveAcceptedViews(file, this->getTypeName());
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief Number of views requested by the user. Once this value is exceeded the Policy is considered complete.
    size_t n_view_requested;

    /// @brief Flag for starting with the `generateUniform` method. If false, the `generateRandom` method is used.
    bool start_uniform;

    /// @brief Flag for using an unordered sampling in `generateUniform`.
    bool unordered;

    /// @brief Radius for sampling positions. Random sampling may sample between `radius` and `radius_max`.
    const float radius, radius_max;

    /// @brief Seed for the random sample. (-1 indicates a random seed is used).
    float seed;

    /// @brief Random sampler utility.
    utilities::RandomSampler<float> sample;

    /// @brief If `unordered` and `start_uniform` are true, then this is the order at which the n_view_requested
    ///        will be returned. If `unordered` is false then this is ordered [0, ... N]. And if `start_uniform`
    ///        is false then this is empty.
    std::vector<size_t> view_order;

    /// @brief Abstracts which method - random or ordered, uniform - is used when `generate` is called.
    std::function<void(Extrinsic&)> call_on_generate;
};


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_SIMPLE_SPHERE_POLICY_HPP
