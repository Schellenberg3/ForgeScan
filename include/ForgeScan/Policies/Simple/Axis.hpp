#ifndef FORGE_SCAN_POLICIES_SIMPLE_AXIS_POLICY_HPP
#define FORGE_SCAN_POLICIES_SIMPLE_AXIS_POLICY_HPP

#include <limits>
#include <functional>

#include "ForgeScan/Policies/Policy.hpp"

#include "ForgeScan/Common/VectorMath.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Random.hpp"


namespace forge_scan {
namespace policies {


class Axis : public Policy
{
public:
    /// @brief Creates a Axis Policy.
    /// @details This policy samples views from around a specified axis, the View Axis. It is
    ///          functionally equal to rotating the a part in front of a stationary camera. 
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param parser Arg Parser with arguments to construct an Axis Policy from.
    /// @return Shared pointer to a Axis Policy.
    static std::shared_ptr<Axis> create(const std::shared_ptr<const data::Reconstruction>& reconstruction,
                                                const utilities::ArgParser& parser)
    {
        float x = parser.getCmdOption<float>("--x", std::nanf("1"));
        float y = parser.getCmdOption<float>("--y", std::nanf("1"));
        float z = parser.getCmdOption<float>("--z", std::nanf("1"));

        // Default is `--z-axis` so we don't actually check this.
        Direction axis = Direction::UnitZ();
        if ( !(std::isnan(x) || std::isnan(y) || std::isnan(z)) )
        {
            axis = Direction(x, y, z);
            axis.normalize();
        }
        else if (parser.cmdOptionExists("--x-axis"))
        {
            axis = Direction::UnitX();
        }
        else if (parser.cmdOptionExists("--y-axis"))
        {
            axis = Direction::UnitY();
        }

        int n_views  = std::max(parser.getCmdOption<int>("--n-views",  10), 1);
        int n_repeat = std::max(parser.getCmdOption<int>("--n-repeat", 1),  1);

        return std::shared_ptr<Axis>(new Axis(reconstruction,
                                              axis, n_views, n_repeat,
                                              parser.getCmdOption<float>("--radius",     2.5),
                                              parser.getCmdOption<float>("--height",     0.0),
                                              parser.getCmdOption<float>("--height_max", 1.0),
                                              parser.cmdOptionExists("--target-center"),
                                              parser.cmdOptionExists("--uniform"),
                                              parser.getCmdOption<float>("--seed", -1)));
    }


protected:
    /// @brief Protected constructor to enforce usage of shared pointer.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param axis    The View Axis about which points are sampled.
    /// @param n_views The number of sampled points.
    /// @param radius  Radius at which points are sampled.
    /// @param height  Hight of the first ring of points.
    /// @param height_max Height of the last ring of points.
    /// @param n_repeat Number or rings to linearly space between `height` and `height_max`.
    /// @param target_center Flag to target all views to the grid center, if true. Otherwise
    ///                      views point at the view axis.
    /// @param uniform  Flag to begin with a uniform sampling strategy. If false or if
    ///                 more than `n_view_requested` have been generated in the uniform manner,
    ///                 then the policy samples randomly between `height` and `height_max`. 
    /// @param seed     Seed for the random generator the Policy uses. Default -1 for a random seed.
    explicit Axis(const std::shared_ptr<const data::Reconstruction>& reconstruction,
                  const Direction& axis,
                  const size_t& n_views,
                  const size_t& n_repeat,
                  const float& radius,
                  const float& height,
                  const float& height_max,
                  const bool& target_center,
                  const bool& uniform,
                  const float& seed)
        : Policy(reconstruction),
          axis(axis),
          n_views(n_views),
          n_repeat(n_repeat),
          n_view_requested(this->n_views * this->n_repeat),
          radius(std::abs(radius)),
          height(std::min(height, height_max)),
          height_max(std::max(height, height_max)),
          start_uniform(uniform),
          target_center(target_center),
          axis_to_grid_center(this->get_axis_to_grid_center()),
          seed(seed),
          sample(this->seed)
    {
        assert(this->n_repeat >= 1 && this->n_views >= 1 &&
               "Policy cannot operate if either n_view or n_repeat equal zero.");

        this->height_linspace.reserve(this->n_repeat);
        float dh = this->n_repeat > 1 ? (this->height_max - this->height) / (this->n_repeat - 1) : 0;
        float  h = this->height;
        for (size_t i = 0; i < this->n_repeat; ++i)
        {
            this->height_linspace[i] = h;
            h += dh;
        }

        this->call_on_generate = this->start_uniform ? std::bind(&Axis::generateUniform,
                                                                  this, std::placeholders::_1) :
                                                       std::bind(&Axis::generateRandom,  
                                                                  this, std::placeholders::_1);


        this->call_for_z_axis = this->target_center ? std::bind(&Axis::targetAtCenter, this,
                                                                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) :
                                                      std::bind(&Axis::targetAtAxis, this,
                                                                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        // This Policy can generate on startup.
        this->generate();
    }


    /// @brief  Calculates the rotation matrix that transforms a point from the View Axis's
    ///         reference frame to the Grid Center's reference frame
    /// @return 
    Rotation get_axis_to_grid_center()
    {
        // Rotation rot_1 = Eigen::Quaternionf().setFromTwoVectors(this->axis, Direction::UnitX()).matrix();
        return Eigen::Quaternionf().setFromTwoVectors(Direction::UnitZ(), this->axis).matrix();
    }


    /// @brief Generate the position for a random view along the axis between the heigh bounds.
    /// @param dest Extrinsic matrix in which the position is stored.
    /// @note When returned `dest` is still in the Axis's reference frame.
    void generateRandom(Extrinsic& dest)
    {
        float theta = this->sample.uniform() * 2 * M_PI;

        dest.translation()      = Point(std::cos(theta), std::sin(theta), 0);
        dest.translation()     *= this->radius;
        dest.translation().z()  = this->sample.uniform(this->height, this->height_max);
    }


    /// @brief Generates the next uniform position for a view along the axis.
    /// @param dest Extrinsic matrix in which the position is stored.
    /// @note When returned `dest` is still in the Axis's reference frame.
    void generateUniform(Extrinsic& dest)
    {
        size_t view_number = this->numAccepted() + this->numRejected();

        if (this->n_view_requested <= view_number)
        {
            // One we have reached the last view we may generate we switch to random sampling.
            this->call_on_generate = std::bind(&Axis::generateRandom, this, std::placeholders::_1);
            this->generateRandom(dest);
            return;
        }

        const float div = (float)view_number / this->n_views;
        const float mod = (float)(view_number % this->n_views);

        const size_t height_idx = static_cast<size_t>(std::max(std::floor(div), 0.0f));
        const float theta = (2.0 * M_PI / (this->n_views - 1)) * mod;

        dest.translation()      = Point(std::cos(theta), std::sin(theta), 0);
        dest.translation()     *= this->radius;
        dest.translation().z() += this->height_linspace[height_idx];
    }


    /// @brief Returns where the Z-axis should point.
    /// @param z_axis[out] The output Z-axis to uses for the view.
    /// @param position Cartesian position of the view, relative to the Grid's reference frame.
    /// @param UNUSED   View target location on the axis.  
    /// @return The z-axis to use for this view.
    void targetAtCenter(Direction& z_axis, const Direction& position, const Direction&)
    {
        z_axis = Direction(-position.x(), -position.y(), -position.z());
        z_axis.normalize();
    }


    /// @brief Returns where the Z-axis should point.
    /// @param z_axis[out] The output Z-axis to uses for the view.
    /// @param position    Cartesian position of the view, relative to the Grid's reference frame.
    /// @param axis_target View target location on the axis.  
    void targetAtAxis(Direction& z_axis, const Direction& position, const Direction& axis_target)
    {
        z_axis = (this->axis_to_grid_center * axis_target).array() - position.array();
        z_axis.normalize();
    }



    // ***************************************************************************************** //
    // *                           PRIVATE VIRTUAL METHOD OVERRIDES                            * //
    // ***************************************************************************************** //


    virtual const std::string& getTypeName() const override final
    {
        const static std::string name("Axis");
        return name;
    }


    virtual void generate() override final
    {
        const Point grid_center = this->reconstruction->grid_properties->getCenter();
     
        Extrinsic extr = Extrinsic::Identity();
        
        // Generate the position in the Axis reference frame and store the target for the axis.
        this->call_on_generate(extr);
        Point target_axis = Point(0.0, 0.0, extr.translation().z());

        // Rotate the position to the Grid Center's reference frame.
        extr.translation() = this->axis_to_grid_center * extr.translation();

        // Calculate what the view's orientation should be.
        Direction z_axis;
        this->call_for_z_axis(z_axis, extr.translation(), target_axis);

        extr.matrix().array().block<3, 1>(0, 0) = this->axis;
        extr.matrix().array().block<3, 1>(0, 1) = z_axis.cross(this->axis);
        extr.matrix().array().block<3, 1>(0, 2) = z_axis;

        // Translate the position to the Grid Lower Bound's reference frame.
        extr.translation() += grid_center;

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

        g_rand_sph.createAttribute("axis",       this->axis);
        g_rand_sph.createAttribute("n_views",    this->n_views);
        g_rand_sph.createAttribute("n_repeat",   this->n_repeat);
        g_rand_sph.createAttribute("radius",     this->radius);
        g_rand_sph.createAttribute("height",     this->height);
        g_rand_sph.createAttribute("height_max", this->height_max);

        g_rand_sph.createAttribute("target_center", static_cast<uint8_t>(this->target_center));
        g_rand_sph.createAttribute("start_uniform", static_cast<uint8_t>(this->start_uniform));
        g_rand_sph.createAttribute("seed", this->seed);
        g_rand_sph.createAttribute("completed", static_cast<uint8_t>(this->isComplete()));

        Policy::saveRejectedViews(file, this->getTypeName());
        Policy::saveAcceptedViews(file, this->getTypeName());
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief The axis about which to sample. This is the Z-axis of the sampling reference frame.
    const Direction axis;

    /// @brief Rotation from the sampling View Axis's reference frame to the Grid Center's reference frame.
    const Rotation axis_to_grid_center;

    /// @brief Number of views per ring.
    const size_t n_views;

    /// @brief Number or rings to repeat along the axis.
    const size_t n_repeat;

    /// @brief Number of views requested by the user. Once this value is exceeded the Policy is considered complete.
    const size_t n_view_requested;

    /// @brief Distance from the axis to sample positions.
    const float radius;

    /// @brief Height of the first ring.
    const float height;

    /// @brief Height of the last ring.
    const float height_max;

    /// @brief Linear spacing of `n_repeat` between `height` and `height_max`.
    std::vector<float> height_linspace;

    /// @brief Flag for starting with the `generateUniform` method. If false, the `generateRandom` method is used.
    const bool start_uniform;

    /// @brief If true will target the center of the grid.
    const bool target_center;

    /// Seed for the random sample. (-1 indicates a random seed is used).
    const float seed;

    /// @brief Random sampler utility.
    utilities::RandomSampler<float> sample;

    /// @brief Abstracts which method - random or ordered, uniform - is used when `generate` is called.
    std::function<void(Extrinsic&)> call_on_generate;

    /// @brief Abstracts which method - the Grid's center or View Axis - is used in `generate` when
    ///        orienting the camera.
    std::function<void(Direction&, const Direction&, const Direction&)> call_for_z_axis;
};


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_SIMPLE_Axis_POLICY_HPP
