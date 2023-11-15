#ifndef FORGE_SCAN_POLICIES_PRECOMPUTED_NORMAL_H
#define FORGE_SCAN_POLICIES_PRECOMPUTED_NORMAL_H

#include <algorithm>

#include "ForgeScan/Policies/Policy.hpp"
#include "ForgeScan/Simulation/Scene.hpp"
#include "ForgeScan/Sensor/DepthImageProccessing.hpp"


namespace forge_scan {
namespace policies {


struct NormalInfo
{
    inline static const std::string type_name =
        "Normal";

    struct Parse
    {
        inline static const std::string file =
            "--file";
    };

    struct Help
    {
        inline static const std::string file =
            "The file location to load a scene and precomputed positions for.";
    };
};


class Normal : public Policy, public simulation::Scene
{
public:
    /// @brief Creates an Normal Policy.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests new
    ///                       views for.
    /// @param parser ArgParser with arguments to construct an Normal Policy from.
    /// @return Shared pointer to a Normal Policy.
    static std::shared_ptr<Normal> create(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                          const utilities::ArgParser& parser)
    {
        return std::shared_ptr<Normal>(new Normal(reconstruction, parser.get<std::filesystem::path>(NormalInfo::Parse::file)));
    }


    static std::shared_ptr<Normal> create(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                          const std::shared_ptr<const sensor::Intrinsics>& intr,
                                          const Extrinsic& grid_lower_bound,
                                          const float& radius,
                                          const float& min_similarity,
                                          const float& n_sample,
                                          const float& n_store,
                                          const float& alpha)
    {
        return std::shared_ptr<Normal>(new Normal(reconstruction, intr, grid_lower_bound, radius, min_similarity, n_sample, n_store, alpha));
    }


    void precomputeViews()
    {
        this->generateNormalScores();
        this->findNextBestViews();
    }


    std::filesystem::path save(std::filesystem::path fpath) const override final
    {
        utilities::validateAndCreateFilepath(fpath, FS_HDF5_FILE_EXTENSION, "PrecomputedViewsNormal", true);

        HighFive::File file(fpath.string(), HighFive::File::Truncate);

        auto g_policy = file.createGroup(FS_HDF5_POLICY_GROUP);
        this->save(file, g_policy);

        return fpath;
    }


    void save(H5Easy::File& file, HighFive::Group& g_policy) const override final
    {
        auto g_normal = g_policy.createGroup(NormalInfo::type_name);
        g_normal.createAttribute("radius",         this->radius);
        g_normal.createAttribute("min_similarity", this->min_similarity);
        g_normal.createAttribute("n_sample",       this->n_sample);
        g_normal.createAttribute("n_store",        this->n_store);
        g_normal.createAttribute("alpha",          this->alpha);
        g_normal.createAttribute("completed",      static_cast<uint8_t>(this->isComplete()));

        Policy::saveViews(file, NormalInfo::type_name, this->precomputed_views, "precomputed");
        Policy::saveRejectedViews(file, NormalInfo::type_name);
        Policy::saveAcceptedViews(file, NormalInfo::type_name);

        // Save information about the assumed location of the reconstruction location and properties
        Scene::writeExtrToHDF5(file, g_normal.getPath(), this->grid_lower_bound);

        g_normal.createAttribute("VoxelGrid Resolution", this->reconstruction->grid_properties->resolution);
        g_normal.createAttribute("VoxelGrid Dimensions", this->reconstruction->grid_properties->dimensions);
        g_normal.createAttribute("VoxelGrid Size",       this->reconstruction->grid_properties->size);

        // Record what meshes were used and how they were posed.
        this->writeMeshesToHDF5(g_normal, file);
    }


    void load(std::filesystem::path fpath) override final
    {
        fpath = std::filesystem::absolute(fpath.make_preferred());

        HighFive::File file(fpath.string(), HighFive::File::ReadOnly);

        auto g_policy = file.getGroup(FS_HDF5_POLICY_GROUP);
        this->load(file, g_policy);
    }


    void load(H5Easy::File& file, HighFive::Group& g_policy)
    {
        auto g_normal = g_policy.getGroup(NormalInfo::type_name);

        this->radius         = g_normal.getAttribute("radius").read<float>();
        this->min_similarity = g_normal.getAttribute("min_similarity").read<float>();
        this->n_sample       = g_normal.getAttribute("n_sample").read<size_t>();
        this->n_store        = g_normal.getAttribute("n_store").read<int>();
        this->alpha          = g_normal.getAttribute("alpha").read<float>();

        Scene::readExtrFromHDF5(file, g_normal.getPath(), this->grid_lower_bound);

        this->accepted_views.clear();
        this->rejected_views.clear();
        this->precomputed_views.clear();

        for (size_t i = 0; i < this->n_store; ++i)
        {
            Extrinsic extr;
            std::stringstream ss;
            ss << g_normal.getPath() << "/precomputed/" << i;
            extr.matrix() = H5Easy::load<Eigen::Matrix4f>(file, ss.str());
            this->precomputed_views.push_back({i, extr});
        }

        // Re-load the meshes to this scene.
        this->readMeshesFromHDF5(g_normal, file);
    }


private:
    Normal(const std::shared_ptr<data::Reconstruction>& reconstruction,
           const std::shared_ptr<const sensor::Intrinsics>& intr,
           const Extrinsic& grid_lower_bound,
           const float& radius,
           const float& min_similarity,
           const float& n_sample,
           const float& n_store,
           const float& alpha)
        : Policy(reconstruction),
          Scene(),
          camera(sensor::Camera::create(intr)),
          grid_lower_bound(grid_lower_bound),
          radius(radius),
          min_similarity(min_similarity),
          n_sample(n_sample),
          n_store(n_store),
          alpha(alpha)
    {

    }


    Normal(const std::shared_ptr<data::Reconstruction>& reconstruction,
           const std::filesystem::path fpath)
        : Policy(reconstruction),
          Scene()
    {
        this->load(fpath);
    }


    /// @brief Samples uniform, ordered positions on the surface of a sphere.
    /// @param dest Extrinsic matrix in which the position is stored.
    void generateUniform(Extrinsic& dest, const int& i)
    {
        // Golden angle in radians. See: https://en.wikipedia.org/wiki/Golden_angle
        static const float golden_angle_radians = M_PI * (std::sqrt(5) - 1);

        // Avoids division by zero errors is n_view_requested is 1.
        static const float nearly_one = 1 - std::numeric_limits<float>::epsilon();

        float y = 1 - (i / ((float)this->n_sample - nearly_one)) * 2;
        float r_y = std::sqrt(1 - y*y);

        float theta = golden_angle_radians * i;

        float x = std::cos(theta) * r_y;
        float z = std::sin(theta) * r_y;

        Point position(x, y, z);
        position *= this->radius;

        dest.translation() = position;
    }


    /// @brief Generates the sample poses and their normal scores.
    void generateNormalScores()
    {
        const Point grid_center = this->reconstruction->grid_properties->getCenter();

        this->score_and_extr.clear();
        this->score_and_extr.reserve(this->n_sample);
        for (size_t i = 0; i < this->n_sample; ++i)
        {
            Extrinsic extr = Extrinsic::Identity();
            this->generateUniform(extr, i);

            extr.translation() += grid_center;
            extr.rotate(vector_math::get_rotation_to_orient_z_axis(extr, grid_center));
            // extr = this->grid_lower_bound * extr;

            this->score_and_extr.push_back({this->scoreNormals(this->grid_lower_bound * extr), extr});
        }
        this->normalizeNormalScores();
    }


    /// @brief Find the normal view score for a camera pose.
    /// @param extr The camera pose to test.
    /// @return A score in the range `[0, width * height]` where a higher score indicates more normals viewed.
    float scoreNormals(const Extrinsic& extr)
    {

        Eigen::RowVector3f axis = extr.rotation().col(2).transpose();
        auto optical_axis = open3d::core::eigen_converter::EigenMatrixToTensor(axis);

        this->camera->resetDepth(1.0f);
        auto results = this->o3d_scene.CastRays(Scene::getCameraRays(this->camera, extr));
        auto normals = results["primitive_normals"];

        normals *= optical_axis[0];
        auto pixel_scores = normals.Sum({2});
        pixel_scores.Abs_();

        auto mask = pixel_scores.Gt(this->min_similarity).To(open3d::core::Dtype::Float32);
        mask.Clip_(0.0f, 1.0f);
        pixel_scores *= mask;

        // Element-wise original normals
        // this->camera->image = open3d::core::eigen_converter::TensorToEigenMatrixXf(pixel_scores);
        // sensor::DepthImageProcessing::imshow(this->camera, true, "pixel_scores");


        return pixel_scores.Sum({0, 1}).Item<float>();
    }


    /// @brief Normalizes each views camera scores so they are in a range of [0 to 1].
    void normalizeNormalScores()
    {
        auto sort_callable = [](const std::pair<float, Extrinsic>& a, const std::pair<float, Extrinsic>& b){ return a.first < b.first; };
        float max_score = std::max_element(this->score_and_extr.begin(), this->score_and_extr.end(), sort_callable)->first;

        auto normalize_callable = [&max_score](std::pair<float, Extrinsic>& a){ a.first /= max_score; };
        std::for_each(this->score_and_extr.begin(), this->score_and_extr.end(), normalize_callable);
    }


    /// @brief Reviews the list of scores and extrinsics to select a set with high values that are
    ///        also distant from each other.
    void findNextBestViews()
    {
        std::vector<int> default_idx(this->n_sample);
        std::iota (std::begin(default_idx), std::end(default_idx), 0);
        std::shuffle(default_idx.begin(), default_idx.end(), utilities::RandomSampler<int>(42).gen);

        this->precomputed_views.clear();

        for (size_t n = 0; n < this->n_store; ++n)
        {
            int best_idx = default_idx[n];
            float best_score = 0;

            for (size_t i = 0; i < this->score_and_extr.size(); ++i)
            {
                if (this->score_and_extr[i].first > 0)
                {
                    float dist_score = scoreViewDistance(score_and_extr[i].second);
                    float score = this->alpha * this->score_and_extr[i].first + (1 - this->alpha) * dist_score;
                    if (score > best_score)
                    {
                        best_score = score;
                        best_idx   = i;
                    }
                }
            }

            this->score_and_extr.at(best_idx).first = -1;
            this->precomputed_views.push_back({this->precomputed_views.size(), this->score_and_extr[best_idx].second});
        }
    }


    /// @brief Compares a view against all already selected views, return ing a score for its similarity to other views
    /// @param extr The view to test against the selected views.
    /// @return A value between 0 and 1 where a higher indicates it is further away from other views.
    float scoreViewDistance(const Extrinsic& extr)
    {
        float min_dist_score = 1;
        for (const auto& view : this->precomputed_views)
        {
            // Since points exist on the same sphere  the similarity their optical axes, oriented to the center, are
            // analogous to the distance. E.g., views far apart will have opposite axes and a negative cosine similarity.
            float dist = (view.second.rotation().col(2).array() * extr.rotation().col(2).array()).sum();
            
            // Similarity is [-1, 1], but we normalize it then invert it. Now, most similar vectors have a score of 0
            // and dissimilar ones a score of 1.
            min_dist_score = std::min(min_dist_score, 1 - ((dist + 1) / 2));
        }
        return min_dist_score;
    }


    // ***************************************************************************************** //
    // *                           PRIVATE VIRTUAL METHOD OVERRIDES                            * //
    // ***************************************************************************************** //


    virtual const std::string& getTypeName() const override final
    {
        return NormalInfo::type_name;
    }


    void print(std::ostream& out) const override final
    {
        out << NormalInfo::type_name << " Policy sampling poses at radius " << this->radius << ".";
    }


    /// @brief Generate call reset the views list to a copy of poses in the precomputed view list.
    virtual void generate() override final
    {
        this->views.clear();
        for (const auto& list_item : this->precomputed_views)
        {
            this->views.push_back(list_item.second);
        }
    }


    bool isComplete() const override final
    {
        return this->numAccepted() + this->numRejected() >= this->n_store;
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    std::shared_ptr<sensor::Camera> camera;

    Extrinsic grid_lower_bound;

    float radius;

    float min_similarity;

    size_t n_sample;

    size_t n_store;

    float alpha;

    std::vector<std::pair<float, Extrinsic>> score_and_extr;

    std::list<std::pair<size_t, Extrinsic>> precomputed_views;
};


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_PRECOMPUTED_NORMAL_H
