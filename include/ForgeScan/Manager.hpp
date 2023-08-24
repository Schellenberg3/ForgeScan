#ifndef DEMOS_CPP_DEMOS_FORGE_SCAN_MANAGER_HPP
#define DEMOS_CPP_DEMOS_FORGE_SCAN_MANAGER_HPP

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <list>
#include <sstream>

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>
#include <highfive/H5Easy.hpp>

#include "ForgeScan/Common/Definitions.hpp"
#include "ForgeScan/Metrics/Constructor.hpp"
#include "ForgeScan/Policies/Constructor.hpp"
#include "ForgeScan/Data/Reconstruction.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Files.hpp"
#include "ForgeScan/Utilities/XDMF.hpp"


namespace forge_scan {


/// @brief Controls the Reconstruction, any Policies, and and Metrics.
class Manager
{
public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Creates a shared pointer to a Manager.
    /// @param parser Arg Parser with arguments to construct Grid Properties from.
    ///               These Grid Properties are utilized by all Reconstruction Voxel Grids.
    ///               See `forge_scan::Grid::Properties` for details.
    /// @return Shared pointer to a Manager.
    static std::shared_ptr<Manager> create(const utilities::ArgParser& parser)
    {
        return std::shared_ptr<Manager>(new Manager(parser));
    }


    /// @brief Creates a shared pointer to a Manager
    /// @param grid_properties Shared, constant pointer to the Grid Properties to use.
    ///                        These Grid Properties are utilized by all Reconstruction Voxel Grids.
    /// @return Shared pointer to a Manager.
    static std::shared_ptr<Manager> create(const std::shared_ptr<const Grid::Properties>& grid_properties)
    {
        return std::shared_ptr<Manager>(new Manager(grid_properties));
    }


    /// @brief Saves the current state of all items (Voxel Grids, Policies, Metrics, etc.) handled by the Manager. 
    /// @param fpath File path and file name for the data.
    /// @note  - If the file path does not already have the `.h5` extension, then this is added.
    /// @note  - If a file name is not provided then this uses a default of `ForgeScan-[TIME STAMP].h5`.
    void save(std::filesystem::path fpath) const
    {
        utilities::checkPathHasFileNameAndExtension(fpath, FS_HDF5_FILE_EXTENSION, "Reconstruction", true);

        try
        {
            HighFive::File file(fpath, HighFive::File::Truncate);
            this->savePolicies(file);
            this->reconstruction->save(file);
            this->saveMetrics(file);
        }
        catch (const HighFive::Exception& e)
        {
            std::stringstream ss("Encountered a HighFive exception while saving file: ");
            ss << e.what();
            throw std::runtime_error(ss.str());
        }
        this->makeXDMF(fpath);
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC POLICY METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Checks if the Manager has a Policy for view selection.
    /// @return True if at least one Policy has been added and may generate views.
    bool hasPolicy() const
    {
        return this->policy_vec.size() > 0 && this->policy_vec[active_policy_idx] != nullptr;
    }


    /// @brief Checks if the Manager has a Policy for view selection at the specified index of the
    ///        policy list.
    /// @param idx The index position for the Policy to check.
    ///            Policies are stored in the order in which they were added.
    /// @return True a Policy exists at the list location and may generate views.
    bool hasPolicy(const size_t& idx) const
    {
        return this->policy_vec.size() > idx && this->policy_vec[idx] != nullptr;
    }


    /// @brief Adds a new Policy option.
    /// @param parser Arg Parser with arguments to construct a new Policy from.
    ///               See `forge_scan::policies::policy_constructor` for details
    /// @return Index position in the Policy list if the Policy was successfully added.
    /// @note   - If `--set-active` is passed as a flag then the new Policy will immediately be
    ///           the active one used for view suggestions.
    /// @throws Any exceptions thrown by `forge_scan::policies::policy_constructor` pass through this.
    size_t policyAdd(const utilities::ArgParser& parser)
    {
        this->policy_vec.push_back(policies::policy_constructor(parser, this->reconstruction));
        this->policy_vec.back()->setup();
        size_t policy_idx = this->policy_vec.size() - 1;
        if (parser.cmdOptionExists("--set-active"))
        {
            this->active_policy_idx = policy_idx;
        }
        return policy_idx;
    }


    /// @brief Changes which Policy is used when selecting views.
    /// @param idx The index position for the Policy to used.
    ///            Policies are stored in the order in which they were added.
    /// @return True if the active Policy was changed. False if there was no change or if there is
    ///         no Policy for the provided index.
    bool policySetActive(const size_t& idx)
    {
        if (idx < this->policy_vec.size() && idx != this->active_policy_idx)
        {
            this->active_policy_idx = idx;
            return true;
        }
        return false;
    }

    
    /// @brief Gets the Policy the Manager is currently using to suggest views.
    /// @return Shared, constant reference to the active Policy.
    /// @throws `std::runtime_error` If no Policies have been added to the Manager.
    std::shared_ptr<const policies::Policy> policyGetActive() const
    {
        this->throwIfNoActivePolicy();
        return this->policy_vec[this->active_policy_idx];
    }


    /// @brief Signals to the active Policy that it should generate a new set of suggested views.
    /// @throws `std::runtime_error` If no Policies have been added to the Manager.
    void policyGenerate()
    {
        this->throwIfNoActivePolicy();
        return this->policyGetActive()->generate();
    }


    /// @brief Queries the active Policy to get its suggested view.
    /// @return Gets a constant reference to the best view suggested by the active Policy.
    /// @throws `std::runtime_error` If no Policies have been added to the Manager.
    /// @throws `std::runtime_error` If the active Policy could not generate a view.
    const Extrinsic& policyGetView()
    {
        this->throwIfNoActivePolicy();
        return this->policyGetActive()->getView();
    }


    /// @brief Signals to the active Policy that the last view it suggested was accepted.
    /// @returns True if the suggested view was accepted. False if there was no view to accept.
    /// @throws `std::runtime_error` If no Policies have been added to the Manager.
    bool policyAcceptView()
    {
        this->throwIfNoActivePolicy();
        bool res = this->policyGetActive()->acceptView(this->policy_total_views);
        if (res)
        {
            ++this->policy_total_views;
        }
        return res;
    }


    /// @brief Signals to the active Policy that the last view it suggested was rejected.
    /// @returns True if the suggested view was rejected. False if there was no view to accept.
    /// @throws `std::runtime_error` If no Policies have been added to the Manager.
    bool policyRejectView()
    {
        this->throwIfNoActivePolicy();
        bool res = this->policyGetActive()->rejectView(this->policy_total_views);
        if (res)
        {
            ++this->policy_total_views;
        }
        return res;
    }


    /// @brief Queries the active Policy to check if it is complete.
    /// @return True if the active Policy believes the Reconstruction to be complete.
    /// @throws `std::runtime_error` If no Policies have been added to the Manager.
    bool policyIsComplete() const
    {
        this->throwIfNoActivePolicy();
        return this->policyGetActive()->isComplete();
    }



    // ***************************************************************************************** //
    // *                             PUBLIC RECONSTRUCTION METHODS                             * //
    // ***************************************************************************************** //


    /// @brief Adds a Voxel Grid data channel to the Reconstruction.
    /// @param parser Arg Parser with arguments to construct a new Voxel Grid from.
    ///               See `forge_scan::data::Reconstruction::addChannel` for details.
    /// @throws `std::invalid_argument` If there is an issue with the Voxel Grid creation process.
    void reconstructionAddChannel(const utilities::ArgParser& parser)
    {
        this->reconstruction->addChannel(parser);
    }


    /// @brief Removes a Voxel Grid data channel from the Reconstruction.
    /// @param name Name of the channel in the Reconstruction's channel dictionary to remove.
    /// @returns True if successful. False if that channel did not exits or if it is owned by a
    ///          Metric or a Policy.
    bool reconstructionRemoveChannel(const std::string& name)
    {
        return this->reconstruction->removeChannel(name);
    }


    /// @brief Updates each Voxel Grid in the Reconstruction based on the provided set of rays.
    /// @param sensed A set of measurements which act as the end points for a collection of rays.
    /// @param extr   Reference frame for the `sensed` measurements and their common origin.
    /// @warning This transforms the sensed points in-place.
    void reconstructionUpdate(PointMatrix& sensed, const Extrinsic& extr)
    {
        this->preUpdate(sensed, extr);
        sensed = extr * sensed.colwise().homogeneous();
        this->reconstruction->update(sensed, extr.translation());
        this->postUpdate();
        ++this->reconstruction_update_count;
    }

    
    /// @brief Queries the Reconstruction for how many times it has been updated with new data.
    /// @return Total number of successful updates the Reconstruction has had. 
    size_t reconstructionGetUpdateCount() const
    {
        return this->reconstruction_update_count;
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC METRIC METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Adds a new Metric to track information about the Reconstruction.
    /// @param parser Arg Parser with arguments to construct a new Metric from.
    ///               See `forge_scan::metrics::metric_constructor` for details.
    /// @throws `std::invalid_argument` If there is an issue with the Metric creation process.
    /// @throws `std::runtime_error` If a Metric of the requested type is already in use.
    void metricAdd(const utilities::ArgParser& parser)
    {
        this->metricAdd(metrics::metric_constructor(parser, this->reconstruction));
    }


    /// @brief Adds a new Metric to track information about the Reconstruction.
    /// @param metric A pre-constructed Metric to add.
    /// @throws `std::runtime_error` If a Metric of the requested type is already in use.
    void metricAdd(std::shared_ptr<metrics::Metric> metric)
    {
        std::string metric_name = metric->getTypeName();
        if (this->metrics_map.count(metric_name) != 0)
        {
            throw std::runtime_error("MetricOfThisTypeAlreadyExists");
        }
        metric->setup();
        this->metrics_map.insert({metric_name, metric});
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //

    /// @brief Shared, constant Grid Properties used by all items the handled by the Manager. 
    const std::shared_ptr<const Grid::Properties> grid_properties;

    /// @brief Stores the Reconstruction class.
    const std::shared_ptr<data::Reconstruction> reconstruction;


private:
    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param parser Arg Parser with arguments to construct Grid Properties from.
    ///               These Grid Properties are utilized by all Reconstruction Voxel Grids.
    ///               See `forge_scan::Grid::Properties` for details.
    explicit Manager(const utilities::ArgParser& parser)
        : grid_properties(Grid::Properties::createConst(parser)),
          reconstruction(data::Reconstruction::create(this->grid_properties))
    {

    }

    /// @brief Private constructor to enforce use of shared pointers.
    /// @param grid_properties Shared, constant pointer to the Grid Properties to use.
    ///                        These Grid Properties are utilized by all Reconstruction Voxel Grids.
    explicit Manager(const std::shared_ptr<const Grid::Properties>& grid_properties)
        : grid_properties(grid_properties),
          reconstruction(data::Reconstruction::create(this->grid_properties))
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

            this->reconstruction->addToXDMF(file, hdf5_fname);

            utilities::XDMF::writeVoxelGridFooter(file);
            utilities::XDMF::writeFooter(file);

            file.close();
        }
        catch (const std::ofstream::failure& e)
        {
            std::stringstream ss("Encountered a std::ofstream failure while creating XDMF file: ");
            ss << e.what();
            throw std::runtime_error(ss.str());
        }
    }



    // ***************************************************************************************** //
    // *                                PRIVATE POLICY METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Gets the Policy the Manager is currently using to suggest views.
    /// @return Shared reference to the active Policy.
    /// @throws `std::runtime_error` If no Policies have been added to the Manager.
    /// @note Unlike the public method, this is used to access a non-constant-qualified pointer
    ///       to the active policy. Many of the Manager's public policy methods use this overload. 
    std::shared_ptr<policies::Policy> policyGetActive()
    {
        this->throwIfNoActivePolicy();
        return this->policy_vec[active_policy_idx];
    }


    /// @brief Prevents calling Policy methods when there is no active Policy.
    /// @throw `std::runtime_error` if `hasPolicy` returns false for the current `active_policy_idx`.
    void throwIfNoActivePolicy() const
    {
        if (!this->hasPolicy(this->active_policy_idx))
        {
            throw std::runtime_error("ManagerHasNoPolicy");
        }
    }


    /// @brief Saves the data for each Policy.
    /// @param file HDF5 file to save Policy data to.
    void savePolicies(HighFive::File& file) const
    {
        auto g_policy = file.createGroup(FS_HDF5_POLICY_GROUP);
        for (const auto& policy : this->policy_vec)
        {
            policy->save(file, g_policy);
        }
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE METIC METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Calls the preUpdate method for each metric.
    /// @param sensed The sensed points passed to `reconstructionUpdate`.
    /// @param extr   The reference frame and common origin for the `sensed` points passed to
    ///               `reconstructionUpdate`.
    void preUpdate(const PointMatrix& sensed, const Extrinsic& extr)
    {
        for (auto& dict_item : this->metrics_map)
        {
            dict_item.second->preUpdate(sensed, extr, this->reconstructionGetUpdateCount());
        }
    }


    /// @brief Calls the postUpdate method for each metric.
    void postUpdate()
    {
        for (auto& dict_item : this->metrics_map)
        {
            dict_item.second->postUpdate(this->reconstructionGetUpdateCount());
        }
    }


    /// @brief Saves the data for each Metric.
    /// @param file HDF5 file to save Metric data to.
    void saveMetrics(HighFive::File& file) const
    {
        for (const auto& dict_item : this->metrics_map)
        {
            dict_item.second->save(file);
        }
    }



    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief Counts the number of times that the update function has been called.
    size_t reconstruction_update_count = 0;

    /// @brief Counts the total views accepted/rejected across all Policies.
    size_t policy_total_views = 0;

    /// @brief List (really vector) for all Policies used to generate views.
    ///        When a new Policy is added, it is appended to the end of this vector.
    std::vector<std::shared_ptr<policies::Policy>> policy_vec;

    /// @brief Index within the list for which Policy is active. Default, and initialized, to 0. 
    size_t active_policy_idx = 0;

    /// @brief Map of Metric type names to Metric. A map ensures we cannot add two Metrics of the
    ///        same type.
    std::map<std::string, std::shared_ptr<metrics::Metric>> metrics_map;
};


} // namespace forge_scan


#endif // DEMOS_CPP_DEMOS_FORGE_SCAN_MANAGER_HPP
