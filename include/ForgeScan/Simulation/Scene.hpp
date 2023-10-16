#ifndef FORGE_SCAN_SIMULATION_SCENE_HPP
#define FORGE_SCAN_SIMULATION_SCENE_HPP

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>
#include <highfive/H5Easy.hpp>

#include <open3d/t/geometry/RaycastingScene.h>

#include "ForgeScan/Common/Definitions.hpp"
#include "ForgeScan/Common/Exceptions.hpp"
#include "ForgeScan/Metrics/GroundTruth/Occupancy.hpp"
#include "ForgeScan/Metrics/GroundTruth/TSDF.hpp"
#include "ForgeScan/Simulation/Constructor.hpp"
#include "ForgeScan/Sensor/Camera.hpp"

#include "ForgeScan/Utilities/Files.hpp"

// Define some helper constants for HDF5.
// These are undefined at the end of this header.
#define FS_HDF5_SCENE_GROUP            "Scene"
#define FS_HDF5_MESHES_GROUP           "Meshes"
#define FS_HDF5_MESHES_EXTR_SUFFIX     "extrinsic"
#define FS_HDF5_MESHES_FILEPATH        "filepath"
#define FS_HDF5_MESHES_SCALE           "scale"


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
    /// @return Shared pointer to a Scene.
    static std::shared_ptr<Scene> create()
    {
        return std::shared_ptr<Scene>(new Scene());
    }

    virtual ~Scene() {}


    /// @brief Saves the Scene. The records the filepath, scale, and pose for each mesh in the Scene.
    /// @param fpath Location to save the HDF5 file.
    /// @returns Full path to the location the file was saved, including name and file extension.
    virtual std::filesystem::path save(std::filesystem::path fpath) const
    {
        utilities::validateAndCreateFilepath(fpath, FS_HDF5_FILE_EXTENSION, "Scene", true);

        HighFive::File file(fpath.string(), HighFive::File::Truncate);

        auto g_scene = file.createGroup(FS_HDF5_SCENE_GROUP);
        this->writeMeshesToHDF5(g_scene, file);

        return fpath;
    }


    /// @brief Loads a stored scene object from disk.
    /// @param fpath Location to save the HDF5 file.
    /// @throws Any exception encountered while reading the HDF5 file.
    virtual void load(std::filesystem::path fpath)
    {
        fpath = std::filesystem::absolute(fpath.make_preferred());

        HighFive::File file(fpath.string(), HighFive::File::ReadOnly);

        auto g_scene  = file.getGroup(FS_HDF5_SCENE_GROUP);
        this->readMeshesFromHDF5(g_scene, file, fpath);
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


    /// @brief Adds a new mesh to the scene.
    /// @param parser ArgParser with parameters for the mesh to add.
    /// @throws InvalidMapKey If no name was provided.
    /// @throws InvalidMapKey If a shape with the same name already exists.
    void add(const utilities::ArgParser& parser)
    {
        this->mesh_list.emplace_back(MeshLoader::create(parser));
        this->o3d_scene.AddTriangles(this->mesh_list.back().second);
    }


    // ***************************************************************************************** //
    // *                      PUBLIC RAYCASTING SCENE AND CAMERA METHODS                       * //
    // ***************************************************************************************** //


    /// @brief Calculates the tensor of rays to cast into the Open3D RaycastingScene.
    /// @param camera Camera intrinsics and pose to use
    /// @return Tensor of shape {width, height, 6} and datatype float 32.
    static open3d::core::Tensor getCameraRays(const std::shared_ptr<sensor::Camera>& camera)
    {
        return Scene::getCameraRays(camera, camera->extr);
    }


    /// @brief Calculates the tensor of rays to cast into the Open3D RaycastingScene.
    /// @param camera Camera intrinsics to use.
    /// @param extr Pose of the camera, relative to the world frame.
    /// @return Tensor of shape {width, height, 6} and datatype float 32.
    static open3d::core::Tensor getCameraRays(const std::shared_ptr<sensor::Camera>& camera, const Extrinsic& extr)
    {
        open3d::core::Tensor rays({static_cast<long>(camera->intr->height), static_cast<long>(camera->intr->width), 6}, open3d::core::Float32);
        Eigen::Map<Eigen::MatrixXf> rays_map(rays.GetDataPtr<float>(), 6, camera->intr->size());

        Eigen::Matrix<float, 6, 1> r;
        r.topRows<3>() = extr.translation();
        int64_t linear_idx = 0;
        for (size_t y = 0; y < camera->intr->height; ++y)
        {
            for (size_t x = 0; x < camera->intr->width; ++x, ++linear_idx)
            {
                Eigen::Vector3f ray_dir = extr.rotation() * camera->getPoint(y, x);
                r.bottomRows<3>() = ray_dir;
                rays_map.col(linear_idx) = r;
            }
        }
        return rays;
    }


    /// @brief Generates a depth image of the Scene for the provided Camera.
    /// @param camera `sensor::Camera`to store information in.
    /// @param camera_pose The reference frame which the camera's extrinsic matrix is relative to.
    ///                    By default this is Identity for the scene's frame. For poses generated by
    ///                    a `Manager` or policies::Policy` this should be the reference frame of the
    ///                    `data::Reconstruction`
    void image(const std::shared_ptr<sensor::Camera>& camera,
               const Extrinsic& camera_pose = Extrinsic::Identity())
    {
        // Clear any previous data and ensure that ray tracing is in the proper units.
        camera->resetDepth(1.0f);

        open3d::core::Tensor rays = Scene::getCameraRays(camera, camera_pose * camera->extr);

        auto result = this->o3d_scene.CastRays(rays);
        camera->image = open3d::core::eigen_converter::TensorToEigenMatrixXf(result["t_hit"]);
        camera->addNoise();
    }



    // ***************************************************************************************** //
    // *                         PUBLIC VOXEL AND GROUND TRUTH METHODS                         * //
    // ***************************************************************************************** //


    /// @brief Gets a list of voxel center location to test for occupancy or distance.
    /// @param grid_properties Size, shape, and resolution of the voxel grid.
    /// @param lower_bound Lower bound location for the grid.
    /// @return A tensor of shape {n, 3} where n is the number of voxels in the grid. The tensor is
    ///         ordered first in x, then y, then z.
    open3d::core::Tensor getVoxelCenters(const std::shared_ptr<const Grid::Properties>& grid_properties,
                                         const Extrinsic& lower_bound) const
    {
        const size_t n_voxels = grid_properties->getNumVoxels();

        open3d::core::Tensor        voxel_centers({static_cast<long>(n_voxels), 3}, open3d::core::Float32);
        Eigen::Map<Eigen::MatrixXf> voxel_centers_map(voxel_centers.GetDataPtr<float>(), 3, n_voxels);

        size_t linear_idx  = 0;
        Point voxel_scan_f = Point::Zero();
        for (size_t z = 0; z < grid_properties->size.z(); ++z)
        {
            for (size_t y = 0; y < grid_properties->size.y(); ++y)
            {
                for (size_t x = 0; x < grid_properties->size.x(); ++x, ++linear_idx)
                {
                    voxel_centers_map.col(linear_idx) = lower_bound * voxel_scan_f.homogeneous();
                    voxel_scan_f.x() += grid_properties->resolution;
                }
                voxel_scan_f.x()  = 0;
                voxel_scan_f.y() += grid_properties->resolution;
            }
            voxel_scan_f.y()  = 0;
            voxel_scan_f.z() += grid_properties->resolution;
        }
        return voxel_centers;
    }


    /// @brief Calculates a `metrics::ground_truth::Occupancy` VoxelGrid.
    /// @param grid_properties Size, shape, and resolution of the voxel grid.
    /// @param lower_bound Lower bound location for the grid.
    /// @return Shared pointer to the VoxelGrid with occupancy labels.
    std::shared_ptr<metrics::ground_truth::Occupancy>
    calculateGroundTruthOccupancy(const std::shared_ptr<const Grid::Properties>& grid_properties,
                                  const Extrinsic& lower_bound)
    {
        const size_t n_voxels = grid_properties->getNumVoxels();

        open3d::core::Tensor voxel_centers = Scene::getVoxelCenters(grid_properties, lower_bound);

        auto result = this->o3d_scene.ComputeOccupancy(voxel_centers, 0, 5).Reshape({1, static_cast<long>(n_voxels)});
        Eigen::MatrixXi result_eigen = open3d::core::eigen_converter::TensorToEigenMatrixXi(result);

        auto true_occupancy = metrics::ground_truth::Occupancy::create(grid_properties);
        for (size_t i = 0; i < n_voxels; ++i)
        {
            true_occupancy->operator[](i) = result_eigen(0, i) == 1 ? VoxelOccupancy::OCCUPIED :
                                                                      VoxelOccupancy::FREE;
        }
        return true_occupancy;
    }


    /// @brief Calculates a `metrics::ground_truth::TSDF` VoxelGrid.
    /// @param grid_properties Size, shape, and resolution of the voxel grid.
    /// @param lower_bound Lower bound location for the grid.
    /// @return Shared pointer to the VoxelGrid with TSDF values.
    std::shared_ptr<metrics::ground_truth::TSDF>
    calculateGroundTruthTSDF(const std::shared_ptr<const Grid::Properties>& grid_properties,
                             const Extrinsic& lower_bound)
    {
        const size_t n_voxels = grid_properties->getNumVoxels();

        open3d::core::Tensor voxel_centers = Scene::getVoxelCenters(grid_properties, lower_bound);

        auto result = this->o3d_scene.ComputeSignedDistance(voxel_centers, 0, 5).Reshape({1, static_cast<long>(n_voxels)});
        Eigen::MatrixXd result_eigen = open3d::core::eigen_converter::TensorToEigenMatrixXd(result);

        auto true_tsdf = metrics::ground_truth::TSDF::create(grid_properties);
        for (size_t i = 0; i < n_voxels; ++i)
        {
            true_tsdf->operator[](i) = result_eigen(0, i);
        }
        return true_tsdf;
    }


protected:
    // ***************************************************************************************** //
    // *                               PROTECTED CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce shared pointer usage.
    explicit Scene()
    {

    }


    /// @brief Writes each mesh filepath, scaling value, and transformation in the HDF5 Scene group.
    /// @param g_scene Reference to the location to store the mesh information at.
    /// @param file Reference to the opened HDF5 file.
    void writeMeshesToHDF5(HighFive::Group& g_scene, HighFive::File& file) const
    {
        auto g_meshes = g_scene.createGroup(FS_HDF5_MESHES_GROUP);

        int n = 0;
        for (const auto& item : this->mesh_list)
        {
            auto g_mesh = g_meshes.createGroup(std::to_string(n++));
            g_mesh.createAttribute(FS_HDF5_MESHES_SCALE, item.first.scale);
            g_mesh.createAttribute(FS_HDF5_MESHES_FILEPATH, item.first.fpath.string());
            Scene::writeExtrToHDF5(file, g_mesh.getPath(), item.first.extr);
        }
    }


    /// @brief Writes each mesh filepath, scaling value, and transformation in the HDF5 Scene group.
    /// @param g_scene Reference to the location to store the mesh information at.
    /// @param file Reference to the opened HDF5 file.
    /// @param fpath Filesystem path to the opened HDF5 file.
    void readMeshesFromHDF5(HighFive::Group& g_scene,  HighFive::File& file, std::filesystem::path fpath)
    {
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

                this->mesh_list.emplace_back( MeshLoader::create(mesh_fpath, extr, fpath.remove_filename(), scale) );
                this->o3d_scene.AddTriangles(this->mesh_list.back().second);
            }
        }
    }



    // ***************************************************************************************** //
    // *                                PROTECTED CLASS MEMBERS                                * //
    // ***************************************************************************************** //


    /// @brief Open3D's raycasting implementation.
    open3d::t::geometry::RaycastingScene o3d_scene;

    /// @brief List of information about the meshes in the scene and the mesh itself.
    std::list<std::pair<MeshInfo, open3d::t::geometry::TriangleMesh>> mesh_list;
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
#undef FS_HDF5_MESHES_GROUP
#undef FS_HDF5_MESHES_EXTR_SUFFIX
#undef FS_HDF5_MESHES_FILEPATH
#undef FS_HDF5_MESHES_SCALE


#endif // FORGE_SCAN_SIMULATION_SCENE_HPP
