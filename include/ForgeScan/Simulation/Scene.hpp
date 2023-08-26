#ifndef FORGE_SCAN_SIMULATION_SCENE_HPP
#define FORGE_SCAN_SIMULATION_SCENE_HPP

#include <algorithm>
#include <filesystem>
#include <map>
#include <memory>
#include <string>

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>

#include "ForgeScan/Common/Definitions.hpp"
#include "ForgeScan/Common/Entity.hpp"
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
#define FS_HDF5_SHAPES_GROUP           "Shapes"
#define FS_HDF5_SHAPES_SUB_GROUP       "/" FS_HDF5_SCENE_GROUP "/" FS_HDF5_SHAPES_GROUP
#define FS_HDF5_GROUND_TRUTH_GROUP     "GroundTruth"
#define FS_HDF5_OCCUPANCY_DSET         "Occupancy"
#define FS_HDF5_TSDF_DSET              "TSDF"


namespace forge_scan {
namespace simulation {


/// @brief A collection of Primitive objects which are imaged together in the same scene.
struct Scene
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @brief Requires access to the private voxelOccupied function.
    friend class metrics::ground_truth::Occupancy;


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


    /// @brief Saves the scene. The records what objects were in the scene and any ground truth
    ///        information that was generated.
    /// @param fpath Location to save the HDF5 file.
    /// @throws Any exception encountered while saving the HDF5 file or XDMF file.
    void save(std::filesystem::path fpath) const
    {
        utilities::checkPathHasFileNameAndExtension(fpath, FS_HDF5_FILE_EXTENSION, "Scene", true);

        HighFive::File file(fpath, HighFive::File::Truncate);
        auto g_scene = file.createGroup(FS_HDF5_SCENE_GROUP);
        H5Easy::dump(file, FS_HDF5_SCAN_LOWER_BOUND_DSET, this->scan_lower_bound.matrix());

        auto g_shape = g_scene.createGroup(FS_HDF5_SHAPES_GROUP);

        for (const auto& dict_item : this->shapes_map)
        {
            auto g_primitive = g_shape.createGroup(dict_item.first);
            dict_item.second->save(g_primitive);
            const std::string dset_path = FS_HDF5_SHAPES_SUB_GROUP "/" + dict_item.first + "/extr";
            H5Easy::dump(file, dset_path, dict_item.second->getExtr().matrix());
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
    }


    /// @brief Loads a stored scene object from disk.
    /// @param fpath Location to save the HDF5 file.
    /// @throws Any exception encountered while reading the HDF5 file.
    void load(const std::filesystem::path& fpath)
    {
        HighFive::File file(fpath, HighFive::File::ReadOnly);
        auto g_scene  = file.getGroup(FS_HDF5_SCENE_GROUP);
        this->scan_lower_bound.matrix() = H5Easy::load<Eigen::Matrix4f>(file, FS_HDF5_SCAN_LOWER_BOUND_DSET);

        auto scene_groups = g_scene.listObjectNames();

        if(std::find(scene_groups.begin(), scene_groups.end(), FS_HDF5_SHAPES_GROUP) != scene_groups.end())
        {
            // Only clear all elements if we make it this far into loading the HDF5.
            this->shapes_map.clear();

            auto g_shapes = g_scene.getGroup(FS_HDF5_SHAPES_GROUP);
            auto primitive_groups = g_shapes.listObjectNames();

            // Begin reading each shape and getting the attributes.
            for (const auto& shape_name : primitive_groups)
            {
                auto g_primitive = g_shapes.getGroup(shape_name);
                const std::string type_name = g_primitive.getAttribute(FS_HDF5_PRIMITIVE_TYPE_NAME_ATTR).read<std::string>();

                // Use a string stream to simulate parsed arguments from a user.
                // Totally not 'efficient' but what is efficiency anyway?
                std::stringstream ss("--name");
                ss << "--name " << shape_name << " --shape " << type_name;
                if (type_name == "Sphere")
                {
                    ss << " --radius " << g_primitive.getAttribute(FS_HDF5_SPHERE_R_ATTR).read<float>();
                }
                else if (type_name == "Box")
                {
                    ss << " --l " << g_primitive.getAttribute(FS_HDF5_BOX_L_ATTR).read<float>();
                    ss << " --w " << g_primitive.getAttribute(FS_HDF5_BOX_W_ATTR).read<float>();
                    ss << " --h " << g_primitive.getAttribute(FS_HDF5_BOX_H_ATTR).read<float>();
                }
                this->add(ss.str());

                Extrinsic extr;
                const std::string dset_path = FS_HDF5_SHAPES_SUB_GROUP "/" + shape_name + "/extr";
                extr.matrix() = H5Easy::load<Eigen::Matrix4f>(file, dset_path);
                this->transform(shape_name, extr, true);
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



    // ***************************************************************************************** //
    // *                               PUBLIC PRIMITIVE METHODS                                * //
    // ***************************************************************************************** //


    /// @brief Adds a new Primitive Shape to the scene.
    /// @param parser Argument parser with parameters for the Primitive Shape.
    /// @throw `std::runtime_error` If no name was provided.
    /// @throw `std::runtime_error` If a shape with the same name already exists.
    void add(const utilities::ArgParser& parser)
    {
        const std::string name = parser.getCmdOption("--name");
        if (name.empty())
        {
            throw std::runtime_error("No name provided for the shape.");
        }
        else if (this->shapes_map.count(name) != 0)
        {
            throw std::runtime_error("A shape named \"" + name + "\" already exists in the Scene.");
        }
        this->shapes_map.insert( {name, primitive_constructor(parser)} );
    }


    /// @brief Removes a Primitive Shape from the Scene.
    /// @param name Name of the Shape to remove.
    /// @return True if an item was removed. False if no Shape with that name exists.
    bool remove(const std::string& name)
    {
        if (this->shapes_map.erase(name) != 0)
        {
            return true;
        }
        return false;
    }


    /// @brief Transforms a Primitive Shape's pose in the Scene.
    /// @param name  Name of the Shape to transform.
    /// @param extr  Transformation to apply to the Shape.
    /// @param world True for world frame transformation. False for body frame. Default false.
    /// @return True if an item was removed. False if no shape with that name exists.
    bool transform(const std::string& name, const Extrinsic& extr, const bool& world = false)
    {
        if (auto search = this->shapes_map.find(name); search != this->shapes_map.end()) {
            if (world)
            {
                search->second->transformWorldFrame(extr);
            }
            else
            {
                search->second->transformBodyFrame(extr);
            }
            return true;
        }
        return false;
    }



    // ***************************************************************************************** //
    // *                                PUBLIC CAMERA METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Takes an image of the scene with the provided Camera.
    /// @param camera Camera to store information in.
    /// @param pose_is_world_frame  If true, treats the pose as relative to the static world frame.
    ///                             If false, treats the pose as relative to the scan lower bounds.
    ///                             Default false.
    /// @note If this class is being used with poses generated by a Manager or Policy, then world
    ///       pose_is_world_frame should be false. That way this function properly places the
    ///       camera relative to the scan_lower_bound as the Policy intended.
    void image(const std::shared_ptr<sensor::Camera>& camera,
               const bool& pose_is_world_frame = false)
    {
        // The camera's origin position in its own reference frame is always (0,0,0).
        static const Point origin_camera_f = Point::Zero();

        // If we are placing the camera relative to the world frame then the pose is what was provided.
        // But if the pose is relative to the scene frame, then transform it to be in the world frame.
        Extrinsic camera_pose = pose_is_world_frame ? camera->getExtr() : this->scan_lower_bound *  camera->getExtr();

        camera->resetDepth();
        for (size_t row = 0; row < camera->intr->height; ++row)
        {
            for (size_t col = 0; col < camera->intr->width; ++col)
            {
                const Point sensed_camera_f = camera->getPoint(row, col);
                float min_scale = 1;
                for (const auto& item : this->shapes_map)
                {
                    float scale = 1;
                    Point origin_shapes_f = item.second->toThisFromOther(origin_camera_f, camera_pose);
                    Point sensed_shapes_f = item.second->toThisFromOther(sensed_camera_f, camera_pose);
                    if (item.second->hit(origin_shapes_f, sensed_shapes_f, scale))
                    {
                        min_scale = std::max(std::min(scale, min_scale), 0.0f);
                    }
                }
                if (min_scale < 1)
                {
                    camera->image(row, col) *= min_scale;
                }
            }
        }
        camera->saturateDepth();
    }



    // ***************************************************************************************** //
    // *                              PUBLIC GROUND TRUTH METHODS                              * //
    // ***************************************************************************************** //


    /// @brief Sets the Grid Properties for all Ground Truth Grids the Scene calculates.
    /// @param grid_properties Shared, constant pointer to the Grid Properties to use.
    void setGridProperties(const std::shared_ptr<const Grid::Properties>& grid_properties)
    {
        this->grid_properties = grid_properties;
    }


    /// @brief Calculates a Ground Truth Occupancy Grid with the Scene's Grid Properties.
    /// @note  The resulting Grid is stored in the Scene's `true_occupancy` attribute.
    void calculateGroundTruthOccupancy()
    {
        if (this->grid_properties.get() == nullptr)
        {
            // Use default Grid Properties if non were set.
            this->grid_properties = Grid::Properties::createConst();
        }
        this->true_occupancy = metrics::ground_truth::Occupancy::create(this->grid_properties);

        const float res = this->true_occupancy->properties->resolution;
        const float half_res = res * 0.5;
        const size_t nz = this->true_occupancy->properties->size.z();
        const size_t ny = this->true_occupancy->properties->size.y();
        const size_t nx = this->true_occupancy->properties->size.x();

        // Current voxel position, relative to scan_lower_bound frame.
        Point voxel_scan_f = Point::Zero();
        size_t n = 0;
        for (size_t z = 0; z < nz; ++z)
        {
            for (size_t y = 0; y < ny; ++y)
            {
                for (size_t x = 0; x < nx; ++x)
                {
                    if (this->voxelOccupied(voxel_scan_f, half_res))
                    {
                        this->true_occupancy->operator[](n) = VoxelOccupancy::OCCUPIED;
                    }
                    else
                    {
                        this->true_occupancy->operator[](n) = VoxelOccupancy::FREE;
                    }
                    ++n;
                    voxel_scan_f.x() += res;
                }
                // Re-set our X-position.
                voxel_scan_f.x() = 0;
                voxel_scan_f.y() += res;
            }
            // Re-set our Y-position.
            voxel_scan_f.y() = 0;
            voxel_scan_f.z() += res;
        }
    }


    /// @brief Calculates a Ground Truth TSDF Grid with the Scene's Grid Properties.
    /// @note  The resulting Grid is stored in the Scene's `true_tsdf` attribute.
    void calculateGroundTruthTSDF()
    {
        if (this->grid_properties.get() == nullptr)
        {
            // Use default Grid Properties if non were set.
            this->grid_properties = Grid::Properties::createConst();
        }
        this->true_tsdf = metrics::ground_truth::TSDF::create(this->grid_properties);

        const float res = this->true_tsdf->properties->resolution;
        const size_t nz = this->true_tsdf->properties->size.z();
        const size_t ny = this->true_tsdf->properties->size.y();
        const size_t nx = this->true_tsdf->properties->size.x();

        // Current voxel position, relative to scan_lower_bound frame.
        Point voxel_scan_f = Point::Zero();
        size_t n = 0;
        for (size_t z = 0; z < nz; ++z)
        {
            for (size_t y = 0; y < ny; ++y)
            {
                for (size_t x = 0; x < nx; ++x)
                {
                    this->true_tsdf->operator[](n) = this->voxelSignedDistance(voxel_scan_f);
                    ++n;
                    voxel_scan_f.x() += res;
                }
                // Re-set our X-position.
                voxel_scan_f.x() = 0;
                voxel_scan_f.y() += res;
            }
            // Re-set our Y-position.
            voxel_scan_f.y() = 0;
            voxel_scan_f.z() += res;
        }
    }


    /// @brief Returns the Ground Truth Occupancy Grid.
    /// @return Shared pointer to the ground truth Occupancy Grid.
    /// @note If the Ground Truth Occupancy Grid has not been calculated already then this will so so.
    std::shared_ptr<metrics::ground_truth::Occupancy> getGroundTruthOccupancy()
    {
        if (this->true_occupancy == nullptr)
        {
            this->calculateGroundTruthOccupancy();
        }
        return this->true_occupancy;
    }


    /// @brief Returns the Ground Truth TSDF Grid.
    /// @return Shared pointer to the ground truth TSDF Grid.
    /// @note If the Ground Truth TSDF Grid has not been calculated already then this will so so.
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
    ///        scan. This is the reference frame the Ground Truth Voxel Grid data implicitly
    ///        uses and which the views generated by a Policy are relative to.
    Extrinsic scan_lower_bound;

    /// @brief Shared, constant pointer to the Grid Properties to use.
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
    /// @throws `std::runtime_error` If any issues are ofstream failures are encountered when
    ///         writing the XDMF file.
    void makeXDMF(std::filesystem::path fpath) const
    {
        const std::string hdf5_fname = fpath.filename();
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
        catch (const std::ofstream::failure& e)
        {
            std::stringstream ss("Encountered a std::ofstream failure while saving file: ");
            ss << e.what();
            throw std::runtime_error(ss.str());
        }
    }



    /*********************************************************************************************/
    /*                             PRIVATE GROUND TRUTH METHODS                                  */
    /*********************************************************************************************/


    /// @brief Checks if the voxel is occupied by any Shapes in the Scene.
    /// @param center   Voxel center location, relative to the scan_lowe_bound frame.
    /// @param half_res Half of the voxel resolution; the distance from the center to any face.
    /// @return True if the voxel is occupied.
    bool voxelOccupied(const Point& center, const float& half_res) const
    {
        Point center_primitive_f;
        for (const auto& item : this->shapes_map)
        {
            if (item.second->isInside(center, this->scan_lower_bound, center_primitive_f))
            {
                // Fully inside at least one shape.
                return true;
            }
            else
            {
                Translation to_surface = (item.second->getNearestSurfacePoint(center_primitive_f) - center_primitive_f).cwiseAbs();
                if ((to_surface.array() < half_res).all())
                {
                    // Clipped by at the shape.
                    return true;
                }
            }
        }
        // Neither inside any shape nor clipped by any shape.
        return false;
    }


    /// @return The signed distance for the voxel center point.

    /// @brief Calculates the signed distance from the voxel to the closest Shape surface in
    ///        the Scene.  
    /// @param center Voxel center location, relative to the scan_lowe_bound frame.
    /// @return Sign distance for the voxel.
    double voxelSignedDistance(const Point& center)
    {
        float dist = -1 * std::numeric_limits<double>::infinity();
        Point center_primitive_f;
        for (const auto& item : this->shapes_map)
        {
            /// TODO: I do not know how to handle the distance when inside multiple arbitrary shapes
            if (item.second->isInside(center, this->scan_lower_bound, center_primitive_f))
            {
                // Fully inside at least one shape is, for now, infinitely inside all shapes.
                return -1 * std::numeric_limits<double>::infinity();
            }
            // If we are outside this item then record its distance.
            float dist_item = item.second->getSignedDistance(center, this->scan_lower_bound);

            // then store the smallest distance we hae seen so far.
            dist = utilities::math::smallest_magnitude(dist, dist_item);
        }
        return dist;
    }



    // ***************************************************************************************** //
    // *                                PRIVATE CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Dictionary of names to Primitive Shapes.
    std::map<std::string, std::shared_ptr<Primitive>> shapes_map;

    /// @brief Shared reference to a Ground Truth Occupancy Grid for the Scene.
    std::shared_ptr<metrics::ground_truth::Occupancy> true_occupancy{nullptr};

    /// @brief Shared reference to a Ground Truth TSDF for the Scene.
    std::shared_ptr<metrics::ground_truth::TSDF> true_tsdf{nullptr};
};


} // namespace simulation
} // namespace forge_scan


#undef FS_HDF5_SCENE_GROUP
#undef FS_HDF5_SCAN_LOWER_BOUND_DSET
#undef FS_HDF5_GRID_SIZE_ATTR
#undef FS_HDF5_GRID_RESOLUTION_ATTR
#undef FS_HDF5_GRID_DIMENSIONS_ATTR
#undef FS_HDF5_SHAPES_GROUP
#undef FS_HDF5_GROUND_TRUTH_GROUP
#undef FS_HDF5_OCCUPANCY_DSET
#undef FS_HDF5_TSDF_DSET

// Undefine the definitions from Primitive.hpp, Box.hpp, and Sphere.hpp:
#undef FS_HDF5_SHAPE_TYPE_NAME_ATTR
#undef FS_HDF5_SPHERE_R_ATTR
#undef FS_HDF5_BOX_L_ATTR
#undef FS_HDF5_BOX_W_ATTR
#undef FS_HDF5_BOX_H_ATTR

#endif // FORGE_SCAN_SIMULATION_SCENE_HPP
