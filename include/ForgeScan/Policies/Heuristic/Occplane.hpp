#ifndef FORGE_SCAN_POLICIES_HEURISTIC_OCCPLANE_HPP
#define FORGE_SCAN_POLICIES_HEURISTIC_OCCPLANE_HPP

#include <algorithm>
#include <limits>
#include <functional>

#include <open3d/geometry/PointCloud.h>

#include "ForgeScan/Policies/Policy.hpp"

#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Random.hpp"


namespace forge_scan {
namespace policies {


struct OccplaneInfo
{
    inline static const std::string type_name =
        "Occplane";

    struct Parse
    {
        inline static const std::string radius =
            "--r";

        inline static const std::string channel =
            "--channel";

        inline static const std::string eps =
            "--eps";

        inline static const std::string min_points =
            "--min-points";

        inline static const std::string keep_top_n =
            "--keep-top-n";

        inline static const std::string complete_after =
            "--complete-after";
    };

    struct Help
    {
        inline static const std::string radius =
            "The distance from between an occplane voxel and its candidate point";

        inline static const std::string channel =
            "The name of an existing channel which supports Occplanes to use";

        inline static const std::string eps =
            "The distance below which two points are considered neighbors";

        inline static const std::string min_points =
            "The minimum number of points to form a cluster";

        inline static const std::string keep_top_n =
            "The maximum number of views to keep in each generation step";

        inline static const std::string complete_after =
            "Policy.isComplete returns true after this many views";
    };

    struct Def
    {
        inline static const float radius =
            2.5;

        inline static const float eps =
            0.5;

        inline static const int min_points =
            6;

        inline static const int keep_top_n =
            3;

        inline static const int complete_after =
            9;
    };
};


class Occplane : public Policy
{
public:
    /// @brief Creates an Occplane Policy.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param parser ArgParser with arguments to construct an Occplane Policy from.
    /// @return Shared pointer to a Occplane Policy.
    static std::shared_ptr<Occplane> create(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                            const utilities::ArgParser& parser)
    {
        return std::shared_ptr<Occplane>(new Occplane(
            reconstruction,
            parser.get<float>(OccplaneInfo::Parse::radius, OccplaneInfo::Def::radius),
            std::abs(parser.get<float>(OccplaneInfo::Parse::eps, OccplaneInfo::Def::eps)),
            std::max(parser.get<int>(OccplaneInfo::Parse::min_points, OccplaneInfo::Def::min_points), 1),
            std::max(parser.get<int>(OccplaneInfo::Parse::keep_top_n, OccplaneInfo::Def::keep_top_n), 1),
            std::max(parser.get<int>(OccplaneInfo::Parse::complete_after, OccplaneInfo::Def::complete_after), 1),
            parser.get(OccplaneInfo::Parse::channel))
        );
    }


private:
    Occplane(const std::shared_ptr<data::Reconstruction>& reconstruction,
             const float& radius,
             const float& eps,
             const int& min_points,
             const int keep_top_n,
             const int complete_after,
             const std::string& use_channel)
        : Policy(reconstruction),
          radius(radius),
          eps(eps),
          min_points(min_points),
          keep_top_n(keep_top_n),
          complete_after(complete_after)
    {
        if (use_channel.empty())
        {
            auto voxel_grid = data::Binary::create(this->reconstruction->grid_properties);
            this->addChannel(voxel_grid, OccplaneInfo::type_name);
            this->binary_grid = voxel_grid;
        }
        else
        {
            auto voxel_grid = this->reconstruction->getChannelRef(use_channel);
            this->binary_grid = std::dynamic_pointer_cast<data::Binary>(voxel_grid);
            if (this->binary_grid == nullptr)
            {
                throw ConstructorError(OccplaneInfo::type_name + " could not  cast grid of " +
                                       voxel_grid->getTypeName() + " to type " +
                                       data::Binary::type_name + ".");
            }
        }
    }


    /// @brief Stores information about a cluster in `generateOccplaneViews`.
    struct Cluster
    {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        int n_points =  0;
        int label    = -1;
    };


    /// @brief Samples a random position on the surface of a sphere.
    /// @param dest Extrinsic matrix in which the position is stored.
    void generateRandom(Extrinsic& dest) const
    {
        float theta, phi;

        utilities::RandomSampler<float> sample;
        sample.sphere(theta, phi, true);
        dest.translation() = vector_math::spherical_to_cartesian(this->radius, theta, phi);
    }


    /// @brief This function creates either the default first view or a random view.
    /// @param force_random Forces the generate function to use `generateRandom`.
    void generateDefault(const bool& force_random = false)
    {
        Extrinsic extr = Extrinsic::Identity();
        if (this->rejected_views.size() > 0 || force_random)
        {
            this->generateRandom(extr);
        }
        else
        {
            extr.translation().z() = this->radius;
        }

        this->views.clear();

        const Point& grid_center = this->reconstruction->grid_properties->getCenter();
        extr.translation() += grid_center;
        extr.rotate(vector_math::get_rotation_to_orient_z_axis(extr, grid_center));

        // For this policy, clear any views before adding the newly generated one.
        this->views.clear();
        this->views.push_back(extr);
    }


    /// @brief Updates the view list based on the occplanes in the reconstruction.
    void generateOccplaneViews()
    {
        std::vector<Eigen::Vector3d> candidate_normals, candidate_points;
        this->binary_grid->updateOccplanes(candidate_points, candidate_normals);

        // Record the number of occplanes found. But set to zero if somehow the number of points and
        // normals are not equal.
        this->n_occplane = candidate_points.size() * (candidate_points.size() == candidate_normals.size());

        // Fallback to default (random) method if there are no identified occplanes.
        if (this->n_occplane == 0)
        {
            this->generateDefault(true);
            return;
        }

        // Turn the centers & normals into the final positions for the candidate points.
        for (size_t i = 0; i < this->n_occplane; ++i)
        {
            candidate_points[i] += this->radius * candidate_normals[i];
        }

        // Labels from the Open3D DBSCAN function are [-1, 0, 1, ..., M]. We use the labels as an index into the clusters
        // vector so we add one because of the first noise label is -1 and and then add one again to get the total size.
        std::vector<int32_t> labels = open3d::geometry::PointCloud(candidate_points).ClusterDBSCAN(this->eps, this->min_points);
        this->n_clusters = *(std::max_element(labels.begin(), labels.end())) + 2;

        // Fallback to default (random) method if is only one cluster and it is noise.
        if (this->n_clusters == 1 && labels.at(0) == -1)
        {
            this->generateDefault(true);
            return;
        }

        // Create the clusters vector and index each occplane point into it, sum points and normals.
        // The normals are inverted to the pose points from the candidate point to the occplane voxel.
        std::vector<Cluster> clusters(this->n_clusters);
        for (size_t i = 0; i < this->n_occplane; ++i)
        {
            // Labels vector is still [-1, 0, 1, ... (n_clusters - 2)]. So we add one
            // to get the index equivelent for the cluster.
            size_t c = labels[i] + 1;
            clusters[c].n_points += 1;
            clusters[c].center   += candidate_points[i];
            clusters[c].normal   -= candidate_normals[i];
        }

        // Label each cluster. Now labels start from 0 rather than -1.
        int c = -1;
        for (auto& vector_item : clusters)
        {
            vector_item.label   = ++c;
            vector_item.center /= vector_item.n_points;
            vector_item.normal /= vector_item.n_points;
        }

        // Sort in order of cluster size unless the cluster is labeled as noise. Once sorted, remove
        // the last element if its label is noise.
        auto sort_clusters = [](const Cluster& a, const Cluster& b) {
            return (a.n_points * (a.label != 0)) > (b.n_points * (b.label != 0));
        };
        std::sort(clusters.begin(), clusters.end(), sort_clusters);
        if (clusters.back().label == 0) { clusters.pop_back(); }
        if (static_cast<int>(clusters.size()) > this->keep_top_n)
        {
            clusters.resize(this->keep_top_n);
        }

        // Transform each cluster into a view and add it to the view list.
        for (const auto& vector_item : clusters)
        {
            Extrinsic extr = Extrinsic::Identity();
            extr.translation() = vector_item.center.cast<float>();
            extr.rotate(vector_math::get_rotation_to_orient_z_axis(extr, (vector_item.center + vector_item.normal).cast<float>()));
            this->views.push_back(extr);
        }
    }



    // ***************************************************************************************** //
    // *                           PRIVATE VIRTUAL METHOD OVERRIDES                            * //
    // ***************************************************************************************** //


    virtual const std::string& getTypeName() const override final
    {
        return OccplaneInfo::type_name;
    }


    void print(std::ostream& out) const override final
    {
        out << OccplaneInfo::type_name << " Policy sampling at radius " << this->radius << ".";
    }


    virtual void generate() override final
    {
        if (this->accepted_views.size() == 0)
        {
            // For now, force this to be random.
            this->generateDefault(true);
        }
        else
        {
            this->generateOccplaneViews();
        }
    }


    bool isComplete() const override final
    {
        const bool at_least_one_non_default_view_accepted = this->numAccepted() > 1;
        const bool no_occplane_in_last_update = this->n_occplane == 0;
        const bool no_occplane_after_at_least_one_update = at_least_one_non_default_view_accepted &&
                                                           no_occplane_in_last_update;
        return no_occplane_after_at_least_one_update ||
               this->numAccepted() + this->numRejected() > (size_t)this->complete_after;
    }


    void save(H5Easy::File& file, HighFive::Group& g_policy) const override final
    {
        auto g_rand_sph = g_policy.createGroup(OccplaneInfo::type_name);
        g_rand_sph.createAttribute("radius",     this->radius);
        g_rand_sph.createAttribute("completed", static_cast<uint8_t>(this->isComplete()));
        Policy::saveRejectedViews(file, OccplaneInfo::type_name);
        Policy::saveAcceptedViews(file, OccplaneInfo::type_name);
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS MEMBERS                                 * //
    // ***************************************************************************************** //

    /// @brief Radius for sampling positions.
    const float radius;

    /// @brief DBSCAN clustering density parameter.
    const float eps;

    /// @brief Minimum number of neighbors for DBSCAN to form a cluster.
    const size_t min_points;

    /// @brief Maximum number of views to keep after each call to generate.
    const int keep_top_n;

    /// @brief Policy is complete after this many views.
    const int complete_after;

    /// @brief Records the number of occplanes and clusters they were sorted into.
    size_t n_occplane = 0, n_clusters = 0;

    /// @brief View of a `data::Binary` voxel grid which the Policy searches for occplanes.
    std::shared_ptr<data::Binary> binary_grid;
};


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_HEURISTIC_OCCPLANE_HPP
