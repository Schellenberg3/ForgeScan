#ifndef FORGE_SCAN_SIMULATION_SCENE_HPP
#define FORGE_SCAN_SIMULATION_SCENE_HPP

#include <algorithm>
#include <filesystem>
#include <map>
#include <memory>
#include <string>

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>
#include <highfive/H5Easy.hpp>

#include <open3d/t/geometry/RaycastingScene.h>

#include "ForgeScan/Common/Definitions.hpp"
#include "ForgeScan/Common/Entity.hpp"
#include "ForgeScan/Common/Exceptions.hpp"
#include "ForgeScan/Metrics/GroundTruth/Occupancy.hpp"
#include "ForgeScan/Metrics/GroundTruth/TSDF.hpp"
#include "ForgeScan/Simulation/Constructor.hpp"
#include "ForgeScan/Sensor/Camera.hpp"

#include "ForgeScan/Utilities/Files.hpp"
#include "ForgeScan/Utilities/XDMF.hpp"

// Define some helper constants for HDF5.
// These are undefined at the end of this header.
#define FS_HDF5_SCENE_GROUP            "Scene"

#define FS_HDF5_SCAN_LOWER_BOUND_DSET  "/" FS_HDF5_SCENE_GROUP "/" "scan_lower_bound"
#define FS_HDF5_GRID_SIZE_ATTR         "size"
#define FS_HDF5_GRID_RESOLUTION_ATTR   "resolution"
#define FS_HDF5_GRID_DIMENSIONS_ATTR   "dimension"

#define FS_HDF5_MESHES_GROUP           "Meshes"
#define FS_HDF5_MESHES_EXTR_SUFFIX     "extrinsic"
#define FS_HDF5_MESHES_FILEPATH        "filepath"
#define FS_HDF5_MESHES_SCALE           "scale"

#define FS_HDF5_GROUND_TRUTH_GROUP     "GroundTruth"
#define FS_HDF5_OCCUPANCY_DSET         "Occupancy"
#define FS_HDF5_TSDF_DSET              "TSDF"


namespace forge_scan {
namespace simulation {


/// @brief A collection of triangle mesh objects which are imaged together in the same scene.
struct Scene
{
    /// @details Required to print the Scene's contents.
    friend std::ostream& operator<<(std::ostream &, const Scene&);

public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Constructor for a shared pointer to a Scene.
    /// @param scan_lower_bound Location of the scan lower bound.
    /// @return Shared pointer to a Scene.
    static std::shared_ptr<Scene> create(const Extrinsic& scan_lower_bound = Extrinsic::Identity())
    {
        return std::shared_ptr<Scene>(new Scene(scan_lower_bound));
    }


    /// @brief Saves the Scene. The records what objects were in the Scene and any ground truth
    ///        information that was generated.
    /// @param fpath Location to save the HDF5 file.
    /// @returns Full path to the location the file was saved, including name and file extension.
    /// @throws Any exception encountered while saving the HDF5 file or XDMF file.
    std::filesystem::path save(std::filesystem::path fpath) const
    {
        utilities::checkPathHasFileNameAndExtension(fpath, FS_HDF5_FILE_EXTENSION, "Scene", true);
        fpath.make_preferred();
        fpath = std::filesystem::absolute(fpath);
        if (!std::filesystem::exists(fpath.parent_path()))
        {
            std::filesystem::create_directories(fpath.parent_path());
        }

        HighFive::File file(fpath.string(), HighFive::File::Truncate);
        auto g_scene = file.createGroup(FS_HDF5_SCENE_GROUP);
        H5Easy::dump(file, FS_HDF5_SCAN_LOWER_BOUND_DSET, this->scan_lower_bound.matrix());

        auto g_meshes = g_scene.createGroup(FS_HDF5_MESHES_GROUP);

        int n = 0;
        for (const auto& item : this->mesh_list)
        {
            auto g_mesh = g_meshes.createGroup(std::to_string(n++));
            g_mesh.createAttribute(FS_HDF5_MESHES_SCALE, item.first.scale);
            g_mesh.createAttribute(FS_HDF5_MESHES_FILEPATH, item.first.fpath.string());
            Scene::writeExtrToHDF5(file, g_mesh.getPath(), item.first.extr);
        }

        if (this->grid_properties)
        {
            auto g_ground_truth = g_scene.createGroup(FS_HDF5_GROUND_TRUTH_GROUP);
            g_ground_truth.createAttribute(FS_HDF5_GRID_SIZE_ATTR,       this->grid_properties->size);
            g_ground_truth.createAttribute(FS_HDF5_GRID_RESOLUTION_ATTR, this->grid_properties->resolution);
            g_ground_truth.createAttribute(FS_HDF5_GRID_DIMENSIONS_ATTR, this->grid_properties->dimensions);

            bool at_least_one_grid = false;
            if (this->true_occupancy.get() != nullptr)
            {
                this->true_occupancy->save(g_ground_truth);
                at_least_one_grid = true;
            }

            if (this->true_tsdf.get() != nullptr)
            {
                this->true_tsdf->save(g_ground_truth);
                at_least_one_grid = true;
            }

            if (at_least_one_grid)
            {
                this->makeXDMF(fpath);
            }
        }
        return fpath;
    }


    /// @brief Loads a stored scene object from disk.
    /// @param fpath Location to save the HDF5 file.
    /// @throws Any exception encountered while reading the HDF5 file.
    void load(std::filesystem::path fpath)
    {
        fpath.make_preferred();
        fpath = std::filesystem::absolute(fpath);
        HighFive::File file(fpath.string(), HighFive::File::ReadOnly);
        auto g_scene  = file.getGroup(FS_HDF5_SCENE_GROUP);
        this->scan_lower_bound.matrix() = H5Easy::load<Eigen::Matrix4f>(file, FS_HDF5_SCAN_LOWER_BOUND_DSET);

        auto scene_groups = g_scene.listObjectNames();

        if(std::find(scene_groups.begin(), scene_groups.end(), FS_HDF5_MESHES_GROUP) != scene_groups.end())
        {
            // Only clear all elements if we make it this far into loading the HDF5.
            this->mesh_list.clear();

            auto g_meshes = g_scene.getGroup(FS_HDF5_MESHES_GROUP);
            auto mesh_groups = g_meshes.listObjectNames();

            // Begin reading each shape and getting the attributes.
            for (const auto& group_name : mesh_groups)
            {
                auto g_mesh = g_meshes.getGroup(group_name);
                   
                float scale = g_mesh.getAttribute(FS_HDF5_MESHES_SCALE).read<float>();
                std::filesystem::path mesh_fpath = std::filesystem::path(g_mesh.getAttribute(FS_HDF5_MESHES_FILEPATH).read<std::string>());

                Extrinsic extr;
                Scene::readExtrFromHDF5(file, g_mesh.getPath(), extr);

                this->mesh_list.emplace_back( Constructor::create(mesh_fpath, extr, fpath.remove_filename(), scale) );
                this->o3d_scene.AddTriangles(this->mesh_list.back().second);
            }
        }

        if(std::find(scene_groups.begin(), scene_groups.end(), FS_HDF5_GROUND_TRUTH_GROUP) != scene_groups.end())
        {
            auto g_ground_truth = g_scene.getGroup(FS_HDF5_GROUND_TRUTH_GROUP);

            GridSize grid_size = g_ground_truth.getAttribute("size").read<GridSize>();
            float res = g_ground_truth.getAttribute("resolution").read<float>();
            this->grid_properties = Grid::Properties::createConst(res, grid_size);


            auto ground_truth_groups = g_ground_truth.listObjectNames();

            if(std::find(ground_truth_groups.begin(), ground_truth_groups.end(), FS_HDF5_OCCUPANCY_DSET) != ground_truth_groups.end())
            {
                std::vector<uint8_t> data = g_ground_truth.getDataSet(FS_HDF5_OCCUPANCY_DSET).read<std::vector<uint8_t>>();
                this->true_occupancy = metrics::ground_truth::Occupancy::create(this->grid_properties, data);
            }

            if(std::find(ground_truth_groups.begin(), ground_truth_groups.end(), FS_HDF5_TSDF_DSET) != ground_truth_groups.end())
            {
                std::vector<double> data = g_ground_truth.getDataSet(FS_HDF5_TSDF_DSET).read<std::vector<double>>();
                this->true_tsdf = metrics::ground_truth::TSDF::create(this->grid_properties, data);
            }
        }
    }


    /// @brief Adds a new mesh to the scene.
    /// @param parser ArgParser with parameters for the mesh to add.
    /// @throws InvalidMapKey If no name was provided.
    /// @throws InvalidMapKey If a shape with the same name already exists.
    void add(const utilities::ArgParser& parser)
    {
        this->mesh_list.emplace_back(Constructor::create(parser));
        this->o3d_scene.AddTriangles(this->mesh_list.back().second);
    }


    // ***************************************************************************************** //
    // *                                PUBLIC CAMERA METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Takes an image of the Scene with the provided Camera.
    /// @param camera `sensor::Camera`to store information in.
    /// @param pose_is_world_frame  If true, treats the pose as relative to the static world frame.
    ///                             If false, treats the pose as relative to the scan lower bounds.
    ///                             Default false.
    /// @note If this class is being used with poses generated by a Manager or Policy, then world
    ///       pose_is_world_frame should be false. That way this function properly places the
    ///       `sensor::Camera`relative to the scan_lower_bound as the Policy intended.
    void image(const std::shared_ptr<sensor::Camera>& camera,
               const bool& pose_is_world_frame = false)
    {
        // Clear any previous data and ensure that raytracing is in the proper units.
        camera->resetDepth(1.0f);

        // If we are placing the camera relative to the world frame then the pose is what was provided.
        // But if the pose is relative to the scene frame, then transform it to be in the world frame.
        Extrinsic camera_pose = pose_is_world_frame ? camera->extr : this->scan_lower_bound * camera->extr;

        open3d::core::Tensor rays({static_cast<long>(camera->intr->height), static_cast<long>(camera->intr->width), 6}, open3d::core::Float32);
        Eigen::Map<Eigen::MatrixXf> rays_map(rays.GetDataPtr<float>(), 6, camera->intr->size());

        Eigen::Matrix<float, 6, 1> r;
        r.topRows<3>() = camera_pose.translation();
        int64_t linear_idx = 0;
        for (size_t y = 0; y < camera->intr->height; ++y)
        {
            for (size_t x = 0; x < camera->intr->width; ++x, ++linear_idx)
            {
                Eigen::Vector3f ray_dir = camera_pose.rotation() * camera->getPoint(y, x);
                r.bottomRows<3>() = ray_dir;
                rays_map.col(linear_idx) = r;
            }
        }

        auto result = this->o3d_scene.CastRays(rays);
        camera->image = open3d::core::eigen_converter::TensorToEigenMatrixXf(result["t_hit"]);
        camera->addNoise();
    }



    // ***************************************************************************************** //
    // *                              PUBLIC GROUND TRUTH METHODS                              * //
    // ***************************************************************************************** //


    /// @brief Sets the `Grid::Properties` for all Ground Truth Grids the Scene calculates.
    /// @param grid_properties Shared, constant pointer to the `Grid::Properties` to use.
    void setGridProperties(const std::shared_ptr<const Grid::Properties>& grid_properties)
    {
        this->grid_properties = grid_properties;
    }


    /// @brief Calculates a `metrics::ground_truth::Occupancy` Grid with the Scene's `Grid::Properties`.
    /// @note  The resulting Grid is stored in the Scene's `true_occupancy` attribute.
    void calculateGroundTruthOccupancy()
    {
        if (this->grid_properties.get() == nullptr)
        {
            this->grid_properties = Grid::Properties::createConst();
        }
        this->true_occupancy = metrics::ground_truth::Occupancy::create(this->grid_properties);

        const size_t n_voxels = this->grid_properties->getNumVoxels();
        open3d::core::Tensor voxel_centers = this->getVoxelCenters();

        auto result = this->o3d_scene.ComputeOccupancy(voxel_centers, 0, 5);
        result = result.Reshape({1, static_cast<long>(n_voxels)});

        Eigen::MatrixXi result_eigen = open3d::core::eigen_converter::TensorToEigenMatrixXi(result);
        for (size_t i = 0; i < n_voxels; ++i)
        {
            this->true_occupancy->operator[](i) = result_eigen(0, i) == 1 ? VoxelOccupancy::OCCUPIED :
                                                                            VoxelOccupancy::FREE;
        }
    }


    /// @brief Calculates a `metrics::ground_truth::TSDF` Grid with the Scene's `Grid::Properties`.
    /// @note  The resulting Grid is stored in the Scene's `true_tsdf` attribute.
    void calculateGroundTruthTSDF()
    {
        if (this->grid_properties.get() == nullptr)
        {
            this->grid_properties = Grid::Properties::createConst();
        }
        this->true_tsdf = metrics::ground_truth::TSDF::create(this->grid_properties);

        const size_t n_voxels = this->grid_properties->getNumVoxels();
        open3d::core::Tensor voxel_centers = this->getVoxelCenters();

        auto result = this->o3d_scene.ComputeSignedDistance(voxel_centers, 0, 5);
        result = result.Reshape({1, static_cast<long>(n_voxels)});

        Eigen::MatrixXd result_eigen = open3d::core::eigen_converter::TensorToEigenMatrixXd(result);
        for (size_t i = 0; i < n_voxels; ++i)
        {
            this->true_tsdf->operator[](i) = result_eigen(0, i);
        }
    }


    /// @brief Returns the `metrics::ground_truth::Occupancy` Grid.
    /// @return Shared pointer to the `metrics::ground_truth::Occupancy` Grid.
    /// @note If the `metrics::ground_truth::Occupancy` Grid has not been calculated already then this will so so.
    std::shared_ptr<metrics::ground_truth::Occupancy> getGroundTruthOccupancy()
    {
        if (this->true_occupancy == nullptr)
        {
            this->calculateGroundTruthOccupancy();
        }
        return this->true_occupancy;
    }


    /// @brief Returns the `metrics::ground_truth::TSDF` Grid.
    /// @return Shared pointer to the `metrics::ground_truth::TSDF` Grid.
    /// @note If the `metrics::ground_truth::TSDF` Grid has not been calculated already then this will so so.
    std::shared_ptr<metrics::ground_truth::TSDF> getGroundTruthTSDF()
    {
        if (this->true_tsdf == nullptr)
        {
            this->calculateGroundTruthTSDF();
        }
        return this->true_tsdf;
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Transformation from the world frame to the lower bound of the Reconstruction
    ///        scan. This is the reference frame the Ground Truth VoxelGrid data implicitly
    ///        uses and which the views generated by a Policy are relative to.
    Extrinsic scan_lower_bound;

    /// @brief Shared, constant pointer to the `Grid::Properties` to use.
    std::shared_ptr<const Grid::Properties> grid_properties{nullptr};


private:
    // ***************************************************************************************** //
    // *                                PRIVATE CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce shared pointer usage.
    /// @param scan_lower_bound Location of the scan lower bound.
    explicit Scene(const Extrinsic& scan_lower_bound)
        : scan_lower_bound(scan_lower_bound)
    {

    }


    /// @brief Writes an XDMF to pair with the HDF5 file for visualizing the data in tools like
    ///        ParaView.
    /// @param fpath File path, with file name, for the HDF5 file.
    /// @throws std::runtime_error If any issues are ofstream failures are encountered when
    ///         writing the XDMF file.
    void makeXDMF(std::filesystem::path fpath) const
    {
        const std::string hdf5_fname = fpath.filename().string();
        fpath.replace_extension(FS_XDMF_FILE_EXTENSION);

        const size_t num_voxels = this->grid_properties->getNumVoxels();

        Point lower = Point::Zero();
        lower.array() -= 0.5 * this->grid_properties->resolution;

        GridSize adjsz = this->grid_properties->size.array() + 1;
        adjsz.reverseInPlace();

        std::ofstream file;
        file.exceptions( std::ofstream::failbit | std::ofstream::badbit );

        try
        {
            file.open(fpath);
            file << std::fixed << std::setprecision(8);
            utilities::XDMF::writeHeader(file);
            utilities::XDMF::writeVoxelGridHeader(
                file, this->grid_properties->resolution,
                adjsz.x(), adjsz.y(), adjsz.z(),
                lower.x(), lower.y(), lower.z()
            );

            if (this->true_occupancy.get() != nullptr)
            {
                utilities::XDMF::writeVoxelGridAttribute(
                    file,
                    this->true_occupancy->getTypeName(),
                    utilities::XDMF::makeDataPath(hdf5_fname, FS_HDF5_SCENE_GROUP, FS_HDF5_GROUND_TRUTH_GROUP, FS_HDF5_OCCUPANCY_DSET),
                    getNumberTypeXDMF(this->true_occupancy->type_id),
                    getNumberPrecisionXDMF(this->true_occupancy->type_id),
                    num_voxels
                );
            }

            if (this->true_tsdf.get() != nullptr)
            {
                utilities::XDMF::writeVoxelGridAttribute(
                    file,
                    this->true_tsdf->getTypeName(),
                    utilities::XDMF::makeDataPath(hdf5_fname, FS_HDF5_SCENE_GROUP, FS_HDF5_GROUND_TRUTH_GROUP, FS_HDF5_TSDF_DSET),
                    getNumberTypeXDMF(this->true_tsdf->type_id),
                    getNumberPrecisionXDMF(this->true_tsdf->type_id),
                    num_voxels
                );
            }

            utilities::XDMF::writeVoxelGridFooter(file);
            utilities::XDMF::writeFooter(file);
            file.close();
        }
        catch (const std::ofstream::failure&)
        {
            throw std::runtime_error("Encountered a std::ofstream failure while saving the Scene XDMF file.");
        }
    }


    /// @brief Writes an Extrinsic eigen matrix to an HDF5 file with HighFive.
    /// @param file HDF5 file to use.
    /// @param mesh_group_path Path to the matrix location.
    /// @param extr The extrinsic matrix to save.
    static void writeExtrToHDF5(HighFive::File& file, const std::string& mesh_group_path, const Extrinsic& extr)
    {
        H5Easy::dump(file, mesh_group_path + "/" FS_HDF5_MESHES_EXTR_SUFFIX, extr.matrix());
    }


    /// @brief Reads an Extrinsic eigen matrix from an HDF5 file with HighFive.
    /// @param file HDF5 file to use.
    /// @param mesh_group_path Path to the matrix location.
    /// @param extr The extrinsic matrix to read.
    static void readExtrFromHDF5(HighFive::File& file, const std::string& mesh_group_path, Extrinsic& extr)
    {
        extr.matrix() = H5Easy::load<Eigen::Matrix4f>(file, mesh_group_path + "/" FS_HDF5_MESHES_EXTR_SUFFIX);
    }



    /*********************************************************************************************/
    /*                             PRIVATE GROUND TRUTH METHODS                                  */
    /*********************************************************************************************/


    /// @brief Gets a list of voxel center location to test for occupancy or distance.
    /// @return A tensor of shape {n, 3} where n is the number of voxels in the grid. The tensor is
    ///         ordered first in x, then y, then z.
    open3d::core::Tensor getVoxelCenters()
    {
        const size_t n_voxels = this->grid_properties->getNumVoxels();

        open3d::core::Tensor        voxel_centers({static_cast<long>(n_voxels), 3}, open3d::core::Float32);
        Eigen::Map<Eigen::MatrixXf> voxel_centers_map(voxel_centers.GetDataPtr<float>(), 3, n_voxels);

        size_t linear_idx  = 0;
        Point voxel_scan_f = Point::Zero();
        for (size_t z = 0; z < this->grid_properties->size.z(); ++z)
        {
            for (size_t y = 0; y < this->grid_properties->size.y(); ++y)
            {
                for (size_t x = 0; x < this->grid_properties->size.x(); ++x, ++linear_idx)
                {
                    voxel_centers_map.col(linear_idx) = this->scan_lower_bound * voxel_scan_f.homogeneous();
                    voxel_scan_f.x() += this->grid_properties->resolution;
                }
                voxel_scan_f.x()  = 0;
                voxel_scan_f.y() += this->grid_properties->resolution;
            }
            voxel_scan_f.y()  = 0;
            voxel_scan_f.z() += this->grid_properties->resolution;
        }

        return voxel_centers;
    }


    // ***************************************************************************************** //
    // *                                PRIVATE CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Open3D's raycasting implementation.
    open3d::t::geometry::RaycastingScene o3d_scene;

    /// @brief List of information about the meshes in the scene and the mesh itself.
    std::list<std::pair<MeshInfo, open3d::t::geometry::TriangleMesh>> mesh_list;

    /// @brief Shared reference to a Ground Truth Occupancy Grid for the Scene.
    std::shared_ptr<metrics::ground_truth::Occupancy> true_occupancy{nullptr};

    /// @brief Shared reference to a Ground Truth TSDF for the Scene.
    std::shared_ptr<metrics::ground_truth::TSDF> true_tsdf{nullptr};
};


/// @brief Writes the contents of the Scene to the output stream.
/// @param out Output stream to write to.
/// @param scene Scene to write out.
/// @return Reference to the output stream.
std::ostream& operator<<(std::ostream &out, const Scene& scene)
{
    if (!scene.mesh_list.empty())
    {
        out << "Scene contains:";
        size_t n = 0;
        for (const auto& item : scene.mesh_list)
        {
            std::string description = item.second.ToString();
            std::replace(description.begin(), description.end(), '\n', ' ');

            std::string center = item.second.GetCenter().ToString();
            center.erase(center.find('\n'));

            out << "\n[" << n << "] Mesh name: " << item.first.fpath.stem() << " centered at " << center
                    << "\n\tFrom file: " << item.first.fpath
                    << "\n\tWith properties:" << description;
            ++n;
        }
    }
    else
    {
        out << "Empty Scene.";
    }
    return out;
}


} // namespace simulation
} // namespace forge_scan


#undef FS_HDF5_SCENE_GROUP
#undef FS_HDF5_SCAN_LOWER_BOUND_DSET
#undef FS_HDF5_GRID_SIZE_ATTR
#undef FS_HDF5_GRID_RESOLUTION_ATTR
#undef FS_HDF5_GRID_DIMENSIONS_ATTR
#undef FS_HDF5_MESHES_GROUP
#undef FS_HDF5_MESHES_EXTR_SUFFIX
#undef FS_HDF5_MESHES_FILEPATH
#undef FS_HDF5_MESHES_SCALE
#undef FS_HDF5_GROUND_TRUTH_GROUP
#undef FS_HDF5_OCCUPANCY_DSET
#undef FS_HDF5_TSDF_DSET


#endif // FORGE_SCAN_SIMULATION_SCENE_HPP
