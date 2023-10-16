#ifndef FORGE_SCAN_SIMULATION_MESH_LOADER_HPP
#define FORGE_SCAN_SIMULATION_MESH_LOADER_HPP

#include <memory>
#include <algorithm>

#include <open3d/core/EigenConverter.h>
#include <open3d/t/geometry/TriangleMesh.h>
#include <open3d/t/io/TriangleMeshIO.h>

#include "ForgeScan/Common/Entity.hpp"


namespace forge_scan {
namespace simulation {


/// @brief Stores extra information about a mesh file.
struct MeshInfo
{
    /// @brief File path to the mesh.
    std::filesystem::path fpath;

    /// @brief Transformation applied to the mesh.
    Extrinsic extr;

    /// @brief Pre-transformation scaling of the mesh about the global origin.
    float scale;
};


/// @brief MeshLoader for loading meshes into a `simulation::Scene` based on ArgParser inputs.
struct MeshLoader
{
    /// @brief Function to read meshes into a `simulation::Scene`.
    /// @param parser Arguments to select the mesh file and describe its transformation.
    /// @return A pair with the MeshInfo and the TriangleMesh object.
    /// @throws ConstructorError if there was an issue loading the mesh
    static std::pair<MeshInfo, open3d::t::geometry::TriangleMesh> create(const utilities::ArgParser& parser)
    {
        Extrinsic extr = Extrinsic::Identity();
        Entity::setRotation(parser, extr);
        Entity::setTranslation(parser, extr);

        std::filesystem::path fpath = parser.get<std::filesystem::path>(Parse::file);
        float scale = parser.get<float>(Parse::scale, Parse::d_scale);

        return MeshLoader::create(fpath, extr, scale);
    }


    /// @brief Function to read meshes into a `simulation::Scene`.
    /// @param fpath File path to the mesh.
    /// @param extr  Extrinsic transformation to apply to the mesh.
    /// @param scale Scaling factor to apply to the mesh. Default 1.
    /// @return A pair with the MeshInfo and the TriangleMesh object.
    /// @throws ConstructorError if there was an issue loading the mesh
    static std::pair<MeshInfo, open3d::t::geometry::TriangleMesh> create(std::filesystem::path fpath,
                                                                         Extrinsic& extr, const float& scale = 1.0f)
    {
        return MeshLoader::create(fpath, extr, std::filesystem::path(), scale);
    }


    /// @brief Function to read meshes into a `simulation::Scene`.
    /// @param fpath File path to the mesh.
    /// @param extr  Extrinsic transformation to apply to the mesh.
    /// @param extra_search_path One additional path to search for the mesh file at.
    /// @param scale Scaling factor to apply to the mesh. Default 1.
    /// @return A pair with the MeshInfo and the TriangleMesh object.
    /// @throws ConstructorError if there was an issue loading the mesh
    static std::pair<MeshInfo, open3d::t::geometry::TriangleMesh> create(std::filesystem::path fpath, Extrinsic& extr,
                                                                         std::filesystem::path extra_search_path,
                                                                         const float& scale = 1.0f)
    {
        const static open3d::core::Tensor origin = open3d::core::Tensor(std::vector<float>{0.0f, 0.0f, 0.0f});

        const static std::filesystem::path share_mesh_path = std::filesystem::path(FORGE_SCAN_MESHES_DIR).make_preferred();
        const static bool share_mesh_path_exists           = std::filesystem::exists(share_mesh_path);

        if (std::filesystem::exists(fpath) == false)
        {
            if (share_mesh_path_exists &&
                std::filesystem::exists(share_mesh_path / fpath))
            {
                fpath = share_mesh_path / fpath;
            }
            else if (extra_search_path.empty() == false         &&
                     std::filesystem::exists(extra_search_path) &&
                     std::filesystem::exists(share_mesh_path / fpath))
            {
                fpath = extra_search_path / fpath;
            }
        }

        if (std::filesystem::exists(fpath) == false || fpath.has_filename() == false)
        {
            throw ConstructorError("Cannot load mesh, no file name provided or file does not exist at: " + fpath.string());
        }

        open3d::t::geometry::TriangleMesh mesh;
        bool read_success = open3d::t::io::ReadTriangleMesh(fpath.string(), mesh);

        if (read_success == false)
        {
            throw ConstructorError("Failed to read mesh file at: " + fpath.string());
        }

        mesh.Scale(scale, origin);
        mesh.Transform(open3d::core::eigen_converter::EigenMatrixToTensor(extr.matrix()));
        return {{fpath.make_preferred(), extr, scale}, mesh};
    }


    /// @brief Returns a string help message for constructing a Policy.
    /// @param parser Arguments to pass determine which help information to print.
    static std::string help([[__maybe_unused__]] const utilities::ArgParser& parser)
    {
        using namespace utilities::strings;
        /// TODO: Return and update this.
        return "Help string for Simulation MeshLoader to be added soon.";
    }


    /// @brief Describes the flags and options that the MeshLoader can parse.
    struct Parse
    {
        /// @brief Option. Describes the location of the mesh file.
        static constexpr char file[] = "--file";

        /// @brief Option. Scaling factor for the mesh.
        static constexpr char scale[] = "--scale";

        /// @brief Default. Scaling factor of 1.
        static constexpr float d_scale = 1.0f;

    };
};


} // namespace primitives
} // namespace forge_scan


#endif // FORGE_SCAN_SIMULATION_MESH_LOADER_HPP
