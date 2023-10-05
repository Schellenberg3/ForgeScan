#ifndef FORGE_SCAN_POLICIES_SIMPLE_AXIS_POLICY_HPP
#define FORGE_SCAN_POLICIES_SIMPLE_AXIS_POLICY_HPP

#include <functional>
#include <limits>
#include <list>

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
    /// @details This Policy samples views from around a specified axis, the View Axis. It is
    ///          functionally equal to rotating the a part in front of a stationary camera.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param parser ArgParser with arguments to construct an Axis Policy from.
    /// @return Shared pointer to a Axis Policy.
    static std::shared_ptr<Axis> create(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                        const utilities::ArgParser& parser)
    {
        // Get the seed (in case we need a random axis).
        float seed = parser.get<float>(Policy::parse_seed, Policy::default_seed);

        // Assume that Z-axis is requested.
        Direction axis = Direction::UnitZ();

        // Look for user-specified axis then check if the a unit axis other than Z was requested.
        float x = parser.get<float>(Axis::parse_x, Axis::default_axis_val);
        float y = parser.get<float>(Axis::parse_y, Axis::default_axis_val);
        float z = parser.get<float>(Axis::parse_z, Axis::default_axis_val);
        if (x != Axis::default_axis_val || y != Axis::default_axis_val ||  z != Axis::default_axis_val)
        {
            axis = Direction(x, y, z);
            axis.normalize();
        }
        else if (parser.has(Axis::parse_x_axis))
        {
            axis = Direction::UnitX();
        }
        else if (parser.has(Axis::parse_y_axis))
        {
            axis = Direction::UnitY();
        }
        else if (parser.has(Axis::parse_random_axis))
        {
            std::random_device rd;
            unsigned int gen_seed = seed > 0 ? static_cast<unsigned int>(seed) : rd();
            std::mt19937 gen(gen_seed);
            std::uniform_real_distribution<float> dist(-1.0, 1.0);
            axis = Direction::NullaryExpr([&](){return dist(gen);});
            axis.normalize();
        }

        int n_views  = std::max(parser.get<int>(Policy::parse_n_views,  Policy::default_n_views), 1);
        int n_repeat = std::max(parser.get<int>(Axis::parse_n_repeat, 1),  1);

        return std::shared_ptr<Axis>(new Axis(reconstruction,
                                              axis, n_views, n_repeat,
                                              parser.get<float>(Axis::parse_r,     Axis::default_r),
                                              parser.get<float>(Axis::parse_h,     Axis::default_h),
                                              parser.get<float>(Axis::parse_h_max, Axis::default_h_max),
                                              parser.has(Axis::parse_target_center),
                                              parser.has(Axis::parse_uniform),
                                              parser.has(Axis::parse_change_random),
                                              seed));
    }


    /// @return Help message for constructing a Axis Policy with ArgParser.
    static std::string helpMessage()
    {
        return "An Axis Policy generates views around a specified axis. The views may be repeated at "
               "different hights along the axis."
               "\nAn Axis Policy may be created with the following arguments:"
               "\n\t" + help_string_basic +
               "\nThe axis to use may be specified with either:"
               "\n\t(1) " + Axis::help_string_1 +
               "\n\t(2) " + Axis::help_string_2 +
               "\nIf the optional arguments are not provided, the default values are:"
               "\n\t" + Axis::default_arguments;
    }

    static const int default_n_repeat;

    static const float default_axis_val, default_r, default_h, default_h_max;

    static const std::string parse_r, parse_h, parse_h_max, parse_target_center,
                             parse_uniform, parse_n_repeat, parse_change_random,
                             parse_x_axis, parse_y_axis, parse_z_axis, parse_random_axis,
                             parse_x, parse_y, parse_z;

    static const std::string help_string_basic, help_string_1, help_string_2, default_arguments;

    static const std::string type_name;

protected:
    /// @brief Protected constructor to enforce usage of shared pointer.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param axis    The View Axis about which points are sampled.
    /// @param n_views The number of sampled points.
    /// @param n_repeat Number or rings to linearly space between `height` and `height_max`.
    /// @param radius  Radius at which points are sampled.
    /// @param height  Hight of the first ring of points.
    /// @param height_max Height of the last ring of points.
    /// @param target_center Flag to target all views to the grid center, if true. Otherwise
    ///                      views point at the view axis.
    /// @param uniform  Flag to begin with a uniform sampling strategy. If false or if
    ///                 more than `n_view_requested` have been generated in the uniform manner,
    ///                 then the Policy samples randomly between `height` and `height_max`.
    /// @param change_random Flag, if true will not use `height`/`height_max` and instead randomly select
    ///                      a new axis to rotate around.
    /// @param seed     Seed for the random generator the Policy uses. Default -1 for a random seed.
    explicit Axis(const std::shared_ptr<data::Reconstruction>& reconstruction,
                  const Direction& axis,
                  const size_t& n_views,
                  const size_t& n_repeat,
                  const float& radius,
                  const float& height,
                  const float& height_max,
                  const bool& target_center,
                  const bool& uniform,
                  const bool& change_random,
                  const float& seed)
        : Policy(reconstruction),
          n_views(n_views),
          n_repeat(n_repeat),
          n_view_requested(this->n_views * this->n_repeat),
          radius(std::abs(radius)),
          height(std::min(height, height_max)),
          height_max(std::max(height, height_max)),
          start_uniform(uniform),
          target_center(target_center),
          change_random(change_random),
          sample(seed)
    {
        assert(this->n_repeat >= 1 && this->n_views >= 1 &&
               "Policy cannot operate if either n_view or n_repeat equal zero.");

        this->axis.push_back(axis);
        this->axis_to_grid_center = this->get_axis_to_grid_center();

        if (this->change_random)
        {
            this->height = 0.0f;
            this->height_max = 0.0f;
        }

        this->height_linspace.reserve(this->n_repeat);
        float dh = this->n_repeat > 1 ? (this->height_max - this->height) / (this->n_repeat - 1) : 0;
        float  h = this->height;
        for (size_t i = 0; i < this->n_repeat; ++i)
        {
            this->height_linspace[i] = h;
            h += dh;
        }

        this->call_on_generate = this->start_uniform ? std::bind(&Axis::generateUniform,
                                                                  this, std::placeholders::_1, std::placeholders::_2) :
                                                       std::bind(&Axis::generateRandom,
                                                                  this, std::placeholders::_1, std::placeholders::_2);


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
        return Eigen::Quaternionf().setFromTwoVectors(Direction::UnitZ(), this->axis.back()).matrix();
    }


    /// @brief Generate the position for a random view along the axis between the heigh bounds.
    /// @param dest Extrinsic matrix in which the position is stored.
    /// @note When returned `dest` is still in the Axis's reference frame.
    void generateRandom(Extrinsic& dest, const size_t&)
    {
        float theta = this->sample.uniform() * 2 * M_PI;

        dest.translation()      = Point(std::cos(theta), std::sin(theta), 0);
        dest.translation()     *= this->radius;
        dest.translation().z()  = this->sample.uniform(this->height, this->height_max);
    }


    /// @brief Generates the next uniform position for a view along the axis.
    /// @param dest Extrinsic matrix in which the position is stored.
    /// @param view_number The view number being generated.
    /// @note When returned `dest` is still in the Axis's reference frame.
    void generateUniform(Extrinsic& dest, const size_t& view_number)
    {
        if (this->n_view_requested <= view_number)
        {
            // One we have reached the last view we may generate we switch to random sampling.
            this->call_on_generate = std::bind(&Axis::generateRandom, this, std::placeholders::_1, std::placeholders::_2);
            this->generateRandom(dest, view_number);
            return;
        }

        const float div = (float)view_number / this->n_views;
        const float mod = (float)(view_number % this->n_views);

        const size_t height_idx = static_cast<size_t>(std::max(std::floor(div), 0.0f));
        const float theta = (2.0 * M_PI / (this->n_views)) * mod;

        dest.translation()      = Point(std::cos(theta), std::sin(theta), 0);
        dest.translation()     *= this->radius;
        dest.translation().z() += this->height_linspace[height_idx];
    }


    /// @brief Returns where the Z-axis should point.
    /// @param [out] z_axis The output Z-axis to uses for the view.
    /// @param position Cartesian position of the view, relative to the Grid's reference frame.
    /// @return The z-axis to use for this view.
    void targetAtCenter(Direction& z_axis, const Direction& position, const Direction&)
    {
        z_axis = Direction(-position.x(), -position.y(), -position.z());
        z_axis.normalize();
    }


    /// @brief Returns where the Z-axis should point.
    /// @param [out] z_axis The output Z-axis to uses for the view.
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
        return Axis::type_name;
    }


    void print(std::ostream& out) const override final
    {
        std::string method = this->start_uniform ? "uniform" : "random";
        out << Axis::type_name << " Policy sampling at around the axis " << this->axis.back().transpose() << " using a " << method
            << " method at a radius of " << this->radius;
        if (this->start_uniform)
        {
            out << " for " << this->n_repeat << " repetitions of " << this->n_views << " views evenly spaced";
        }
        else
        {
            out << " for " << this->n_view_requested << " views";
        }
        out << " between the heights " << this->height << " and " << this->height_max;
    }

    virtual void generate() override final
    {
        const Point grid_center = this->reconstruction->grid_properties->getCenter();

        Extrinsic extr = Extrinsic::Identity();

        size_t view_number = this->numAccepted() + this->numRejected();
        if (this->change_random &&
            view_number > 0 &&
            view_number % this->n_views == 0)
        {
            this->axis.push_back(Direction::NullaryExpr([this](){return this->sample.uniform();}));
            this->axis.back().normalize();
            this->axis_to_grid_center = this->get_axis_to_grid_center();
        }

        // Generate the position in the Axis reference frame and store the target for the axis.
        this->call_on_generate(extr, view_number);
        Point target_axis = Point(0.0, 0.0, extr.translation().z());

        // Rotate the position to the Grid Center's reference frame.
        extr.translation() = this->axis_to_grid_center * extr.translation();

        // Calculate what the view's orientation should be.
        Direction z_axis;
        this->call_for_z_axis(z_axis, extr.translation(), target_axis);

        extr.matrix().array().block<3, 1>(0, 0) = this->axis.back();
        extr.matrix().array().block<3, 1>(0, 1) = z_axis.cross(this->axis.back());
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
        auto g_rand_sph = g_policy.createGroup(Axis::type_name);

        g_rand_sph.createAttribute("n_views",    this->n_views);
        g_rand_sph.createAttribute("n_repeat",   this->n_repeat);
        g_rand_sph.createAttribute("radius",     this->radius);
        g_rand_sph.createAttribute("height",     this->height);
        g_rand_sph.createAttribute("height_max", this->height_max);

        g_rand_sph.createAttribute("target_center", static_cast<uint8_t>(this->target_center));
        g_rand_sph.createAttribute("start_uniform", static_cast<uint8_t>(this->start_uniform));
        g_rand_sph.createAttribute("seed", this->sample.seed);
        g_rand_sph.createAttribute("completed", static_cast<uint8_t>(this->isComplete()));

        int n = 0;
        const std::string hdf5_data_root = "/" FS_HDF5_POLICY_GROUP "/" + Axis::type_name + "/axis";
        std::stringstream ss;
        for (const auto& ax: this->axis)
        {
            ss << hdf5_data_root << "/" << n;
            H5Easy::dump(file, ss.str(), ax);
            ss.str(std::string());
            ++n;
        }

        Policy::saveRejectedViews(file, Axis::type_name);
        Policy::saveAcceptedViews(file, Axis::type_name);
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief list of the axis about which to sample. The last item is the current axis.
    ///        This is the Z-axis of the sampling reference frame.
    std::list<Direction> axis;

    /// @brief Rotation from the sampling View Axis's reference frame to the Grid Center's reference frame.
    Rotation axis_to_grid_center;

    /// @brief Number of views per ring.
    const size_t n_views;

    /// @brief Number or rings to repeat along the axis.
    const size_t n_repeat;

    /// @brief Number of views requested by the user. Once this value is exceeded the Policy is considered complete.
    const size_t n_view_requested;

    /// @brief Distance from the axis to sample positions.
    const float radius;

    /// @brief Height of the first ring.
    float height;

    /// @brief Height of the last ring.
    float height_max;

    /// @brief Linear spacing of `n_repeat` between `height` and `height_max`.
    std::vector<float> height_linspace;

    /// @brief Flag for starting with the `generateUniform` method. If false, the `generateRandom` method is used.
    const bool start_uniform;

    /// @brief If true will target the center of the grid.
    const bool target_center;

    /// @brief If true will target the center of the grid.
    const bool change_random;

    /// @brief Random sampler utility.
    utilities::RandomSampler<float> sample;

    /// @brief Abstracts which method - random or ordered, uniform - is used when `generate` is called.
    std::function<void(Extrinsic&, const size_t&)> call_on_generate;

    /// @brief Abstracts which method - the Grid's center or View Axis - is used in `generate` when
    ///        orienting the camera.
    std::function<void(Direction&, const Direction&, const Direction&)> call_for_z_axis;
};


/// @brief String for the class name.
const std::string Axis::type_name = "Axis";

/// @brief Default number of repetitions about the sampling axis.
const int Axis::default_n_repeat = 1;

/// @brief Default value for each unit direction of a user-specified axis.
const float Axis::default_axis_val = 0.0f;

/// @brief Default distance from the sampling axis to generate views at.
const float Axis::default_r = 2.5;

/// @brief Default minimum and maximum heights along sampling axis to generate views at.
const float Axis::default_h = 0.0f, Axis::default_h_max = 2.5f;

/// @brief ArgParser key for the distance from the sampling axis to generate views at.
const std::string Axis::parse_r = "--r";

/// @brief ArgParser key for the minimum and maximum heights along sampling axis to generate views at.
const std::string Axis::parse_h = "--h", Axis::parse_h_max = "--h-max";

/// @brief ArgParser flag to always point the sensor at the origin of the Grid. If not provided
///        the sensor is pointed at the axis at the same height as the sensor.
const std::string Axis::parse_target_center = "--target-center";

/// @brief ArgParser flag to sample in uniform rotations around the axis.
const std::string Axis::parse_uniform = "--uniform";

/// @brief ArgParser key for the number of rotations to make about the axis. Rotations are
///        linearly spaced between `h` and `h_max`
const std::string Axis::parse_n_repeat = "--n-repeat";

/// @brief ArgParser flag for how many views about and random axis before changing the random axis.
const std::string Axis::parse_change_random = "--change-random";

/// @brief ArgParser flags to use the specified cartesian axis (in the Grid's frame) as the rotation axis.
const std::string Axis::parse_x_axis = "--x-axis",
                  Axis::parse_y_axis = "--y-axis",
                  Axis::parse_z_axis = "--z-axis",
                  Axis::parse_random_axis = "--random-axis";

/// @brief ArgPArser keys for a user-defined rotation axis (in the Grid's frame).
const std::string Axis::parse_x = "--x",
                  Axis::parse_y = "--y",
                  Axis::parse_z = "--z";


/// @brief String explaining the non axis-related arguments this class accepts when using a cartesian axis.
const std::string Axis::help_string_basic =
    "["  + Policy::parse_n_views + " <views per rotation>]" +
    " [" + Axis::parse_n_repeat  + " <repetitions>]" +
    " [" + Axis::parse_r         + " <radius>]" +
    " [" + Axis::parse_h         + " <starting height>]" +
    " [" + Axis::parse_h_max     + " <maximum height>]" +
    " [" + Policy::parse_seed    + " <RNG seed>]" +
    " [" + Axis::parse_uniform + "] [" + Axis::parse_target_center + "] [" + Axis::parse_change_random + "]";

/// @brief String explaining what arguments this class accepts when using a cartesian axis.
const std::string Axis::help_string_1 =
    "<" + Axis::parse_x_axis + " | " +
          Axis::parse_y_axis + " | " +
          Axis::parse_z_axis + " | " +
          Axis::parse_random_axis + ">";


/// @brief String explaining what arguments this class accepts when using a user-specified axis.
const std::string Axis::help_string_2 =
    "[" + Axis::parse_x      + " <axis X component>]" +
    " [" + Axis::parse_y      + " <axis Y component>]" +
    " [" + Axis::parse_z      + " <axis Z component>]";

/// @brief String explaining what this class's default parsed values are.
const std::string Axis::default_arguments =
    Policy::parse_type + " Axis " + Axis::parse_z_axis + " " + Policy::parse_n_views + " " +
    std::to_string(Policy::default_n_views) + " " + Axis::parse_r + " " + std::to_string(Axis::default_r);

} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_SIMPLE_Axis_POLICY_HPP
