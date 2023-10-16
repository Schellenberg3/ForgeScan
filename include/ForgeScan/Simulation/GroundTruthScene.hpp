#ifndef FORGE_SCAN_SIMULATION_GROUND_TRUTH_SCENE_H
#define FORGE_SCAN_SIMULATION_GROUND_TRUTH_SCENE_H

#include "ForgeScan/Simulation/Scene.hpp"
#include "ForgeScan/Utilities/XDMF.hpp"


// Define some helper constants for HDF5.
// These are undefined at the end of this header.
#define FS_HDF5_GROUND_TRUTH_SCENE_GROUP  "GroundTruthScene"
#define FS_HDF5_GRID_SIZE_ATTR            "size"
#define FS_HDF5_GRID_RESOLUTION_ATTR      "resolution"
#define FS_HDF5_GRID_DIMENSIONS_ATTR      "dimension"
#define FS_HDF5_GROUND_TRUTH_GROUP        "GroundTruth"
#define FS_HDF5_OCCUPANCY_DSET            "Occupancy"
#define FS_HDF5_TSDF_DSET                 "TSDF"


namespace forge_scan {
namespace simulation {


struct GroundTruthScene : public Scene
{
    /// @brief Constructor for a shared pointer to a GroundTruthScene.
    /// @param grid_lower_bound Location of the voxel grid's lower bound.
    /// @param grid_properties Shared properties for the ground truth grid created by this class.
    /// @return Shared pointer to a GroundTruthScene.
    static std::shared_ptr<GroundTruthScene>
    create(const Extrinsic& grid_lower_bound = Extrinsic::Identity(),
           const std::shared_ptr<const Grid::Properties>& grid_properties = Grid::Properties::createConst())
    {
        return std::shared_ptr<GroundTruthScene>(new GroundTruthScene(grid_lower_bound, grid_properties));
    }


    /// @brief Saves the GroundTruthScene. The records what objects were in the GroundTruthScene
    ///        and any ground truth information that was generated.
    /// @param fpath Location to save the HDF5 file.
    /// @returns Full path to the location the file was saved, including name and file extension.
    /// @throws Any exception encountered while saving the HDF5 file or XDMF file.
    virtual std::filesystem::path save(std::filesystem::path fpath) const override final
    {
        utilities::validateAndCreateFilepath(fpath, FS_HDF5_FILE_EXTENSION, "Scene", true);

        HighFive::File file(fpath.string(), HighFive::File::Truncate);
        auto g_scene = file.createGroup(FS_HDF5_GROUND_TRUTH_SCENE_GROUP);

        this->writeMeshesToHDF5(g_scene, file);
        this->writeGridsToHDF5(g_scene, fpath);
        Scene::writeExtrToHDF5(file, g_scene.getPath(), this->grid_lower_bound);

        return fpath;
    }


    /// @brief Loads a stored scene object from disk.
    /// @param fpath Location to save the HDF5 file.
    /// @throws Any exception encountered while reading the HDF5 file.
    virtual void load(std::filesystem::path fpath) override final
    {
        fpath.make_preferred();
        fpath = std::filesystem::absolute(fpath);
        HighFive::File file(fpath.string(), HighFive::File::ReadOnly);
        auto g_scene = file.getGroup(FS_HDF5_GROUND_TRUTH_SCENE_GROUP);

        this->readMeshesFromHDF5(g_scene, file, fpath);
        this->readGridsFromHDF5(g_scene);
        Scene::readExtrFromHDF5(file, g_scene.getPath(), this->grid_lower_bound);
    }


    /// @brief Writes the ground truth voxel grid infomation into a HDF5 file.
    /// @param g_scene Group to write the grid properties, lower bound, and Occupancy or TSDF data in.
    /// @param fpath Filepath for the HDF5 file.
    void writeGridsToHDF5(HighFive::Group& g_scene, std::filesystem::path fpath) const
    {
        auto g_ground_truth = g_scene.createGroup(FS_HDF5_GROUND_TRUTH_GROUP);
        g_ground_truth.createAttribute(FS_HDF5_GRID_SIZE_ATTR,       this->grid_properties->size);
        g_ground_truth.createAttribute(FS_HDF5_GRID_RESOLUTION_ATTR, this->grid_properties->resolution);
        g_ground_truth.createAttribute(FS_HDF5_GRID_DIMENSIONS_ATTR, this->grid_properties->dimensions);

        std::string occupancy_dset_path, tsdf_dset_path;

        if (this->true_occupancy) {
            occupancy_dset_path = this->true_occupancy->save(g_ground_truth).getPath();
        }

        if (this->true_tsdf) {
            tsdf_dset_path = this->true_tsdf->save(g_ground_truth).getPath();
        }

        if (this->true_occupancy || this->true_tsdf) {
            this->makeXDMF(fpath, occupancy_dset_path, tsdf_dset_path);
        }
    }


    /// @brief Reads the ground truth voxel grid infomation from a HDF5 file.
    /// @param g_scene Group to read the grid properties, lower bound, and Occupancy or TSDF data from.
    void readGridsFromHDF5(HighFive::Group& g_scene)
    {
        auto scene_groups = g_scene.listObjectNames();
        if(std::find(scene_groups.begin(), scene_groups.end(), FS_HDF5_GROUND_TRUTH_GROUP) != scene_groups.end())
        {
            auto g_ground_truth = g_scene.getGroup(FS_HDF5_GROUND_TRUTH_GROUP);

            GridSize grid_size = g_ground_truth.getAttribute(FS_HDF5_GRID_SIZE_ATTR).read<GridSize>();
            float res          = g_ground_truth.getAttribute(FS_HDF5_GRID_RESOLUTION_ATTR).read<float>();
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


    /// @brief Gets the scene's occupancy for the grid location and properties.
    /// @return Shared pointer to the `metric::ground_truth::Occupancy` grid
    std::shared_ptr<forge_scan::metrics::ground_truth::Occupancy> getGroundTruthOccupancy()
    {
        if (this->true_occupancy == nullptr) {
            this->calculateGroundTruthOccupancy();
        }
        return this->true_occupancy;
    }


    /// @brief Calculates the scene's occupancy for the scan location.
    void calculateGroundTruthOccupancy()
    {
        this->true_occupancy = Scene::calculateGroundTruthOccupancy(this->grid_properties, this->grid_lower_bound);
    }


    /// @brief Gets the scene's occupancy for the grid location and properties.
    /// @return Shared pointer to the `metric::ground_truth::TSDF` grid
    std::shared_ptr<forge_scan::metrics::ground_truth::TSDF> getGroundTruthTSDF()
    {
        if (this->true_tsdf == nullptr) {
            this->calculateGroundTruthTSDF();
        }
        return this->true_tsdf;
    }


    /// @brief Calculates the scene's TSDF for the scan location.
    void calculateGroundTruthTSDF()
    {
        this->true_tsdf = Scene::calculateGroundTruthTSDF(this->grid_properties, this->grid_lower_bound);
    }


    /// @brief Transformation from the world frame to the lower bound of the Reconstruction
    ///        scan. This is the reference frame the Ground Truth VoxelGrid data implicitly
    ///        uses and which the views generated by a Policy are relative to.
    Extrinsic grid_lower_bound;

    /// @brief Shared, constant pointer to the `Grid::Properties` to use.
    std::shared_ptr<const Grid::Properties> grid_properties;

    /// @brief Shared reference to a Ground Truth Occupancy Grid for the Scene.
    std::shared_ptr<metrics::ground_truth::Occupancy> true_occupancy{nullptr};

    /// @brief Shared reference to a Ground Truth TSDF for the Scene.
    std::shared_ptr<metrics::ground_truth::TSDF> true_tsdf{nullptr};


protected:
    /// @brief Private constructor to enforce shared pointer usage.
    GroundTruthScene(const Extrinsic& grid_lower_bound,
                     const std::shared_ptr<const Grid::Properties>& grid_properties)
        : grid_lower_bound(grid_lower_bound),
          grid_properties(grid_properties)
    {

    }


    /// @brief Writes an XDMF to pair with the HDF5 file for visualizing the data in tools like
    ///        ParaView.
    /// @param fpath File path, with file name, for the HDF5 file.
    /// @throws std::runtime_error If any issues are ofstream failures are encountered when
    ///         writing the XDMF file.
    void makeXDMF(std::filesystem::path fpath, const std::string& occupancy_dset_path = std::string(),
                  const std::string& tsdf_dset_path = std::string()) const
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

            if (this->true_occupancy != nullptr && occupancy_dset_path.empty() == false)
            {
                utilities::XDMF::writeVoxelGridAttribute(
                    file,
                    this->true_occupancy->getTypeName(),
                    utilities::XDMF::makeDataPath(hdf5_fname, occupancy_dset_path),
                    getNumberTypeXDMF(this->true_occupancy->type_id),
                    getNumberPrecisionXDMF(this->true_occupancy->type_id),
                    num_voxels
                );
            }

            if (this->true_tsdf != nullptr && tsdf_dset_path.empty() == false)
            {
                utilities::XDMF::writeVoxelGridAttribute(
                    file,
                    this->true_tsdf->getTypeName(),
                    utilities::XDMF::makeDataPath(hdf5_fname, tsdf_dset_path),
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
};


#undef FS_HDF5_GROUND_TRUTH_SCENE_GROUP
#undef FS_HDF5_GRID_SIZE_ATTR
#undef FS_HDF5_GRID_RESOLUTION_ATTR
#undef FS_HDF5_GRID_DIMENSIONS_ATTR
#undef FS_HDF5_GROUND_TRUTH_GROUP
#undef FS_HDF5_OCCUPANCY_DSET
#undef FS_HDF5_TSDF_DSET


} // namespace simulation
} // namespace forge_scan


#endif // FORGE_SCAN_SIMULATION_GROUND_TRUTH_SCENE_H
