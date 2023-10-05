#ifndef FORGE_SCAN_RECONSTRUCTION_RECONSTRUCTION_HPP
#define FORGE_SCAN_RECONSTRUCTION_RECONSTRUCTION_HPP

#include <map>
#include <string>

#include "ForgeScan/Common/RayTrace.hpp"
#include "ForgeScan/Data/VoxelGrids/Constructor.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"


namespace forge_scan {

    // Forward definition to allow friend access.
    class Manager;

namespace metrics {

    // Forward definition to allow friend access.
    class Metric;

} // namespace metrics

namespace policies {

    // Forward definition to allow friend access.
    class Policy;

} // namespace policies
} // namespace forge_scan


namespace forge_scan {
namespace data {


/// @brief Contains, manages, and updates the measured data to represent a a 3D scene.
/// @note  The `Reconstruction` class implicitly uses the origin of the `Grid::Properties` it was
///        instantiated with as its reference frame. It is required that any 3D data passed in has
///        was already transformed into this frame. Generally a Manager class handles this.
class Reconstruction
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @details Required add/remove channels, call the save method, and call the update method.
    friend class forge_scan::Manager;

    /// @details Required to add/remove Metric-specific channels.
    friend class metrics::Metric;

    /// @details Required to add/remove Policy-specific channels.
    friend class policies::Policy;


public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Creates a shared pointer to a Reconstruction.
    /// @param grid_properties Shared, constant pointer to the `Grid::Properties` to use.
    ///                        These `Grid::Properties` are utilized by all VoxelGrids.
    /// @return Shared pointer to a Reconstruction.
    static std::shared_ptr<Reconstruction> create(std::shared_ptr<const Grid::Properties> grid_properties)
    {
        return std::shared_ptr<Reconstruction>(new Reconstruction(grid_properties));
    }


    /// @brief Updates each VoxelGrid based on the provided set of rays.
    /// @param sensed_points A set of measurements which act as the end points for a collection of rays.
    /// @param origin Common origin of the sensed points.
    /// @note Both `sensed` and `origin` are assumed to be in the Reconstruction's reference frame.
    void update(const PointMatrix& sensed_points, const Point& origin)
    {
        for (const auto& sensed : sensed_points.colwise())
        {
            if(get_ray_trace(this->ray_trace, sensed, origin, this->grid_properties,
                             this->min_dist_min, this->max_dist_max))
            {
                for (auto it = this->ray_trace->first_above(0.0f); it != this->ray_trace->end(); ++it)
                {
                    this->data_seen->operator[](it->i) = true;
                }

                for (const auto& item : this->channels)
                {
                    item.second->update(this->ray_trace);
                }
            }
        }
        for (const auto& item : this->channels)
        {
            item.second->postUpdate();
        }
    }


    /// @brief Adds a VoxelGrid data channel to the Reconstruction.
    /// @param parser ArgParser with arguments to construct a new VoxelGrid from.
    ///               See `forge_scan::data::Reconstruction::addChannel` for details.
    /// @throws InvalidMapKey If no name was provided for the channel.
    /// @throws InvalidMapKey If there is already a channel with that name.
    /// @throws ReservedMapKey if the channel name is reserved for Metrics or Policies.
    /// @throws Any exceptions thrown by `Constructor::create` pass through this.
    void addChannel(const utilities::ArgParser& parser)
    {
        std::string channel_name = parser.get(Reconstruction::parse_name);
        if (channel_name.empty())
        {
            throw InvalidMapKey::NoNameProvided();
        }
        else if (this->channels.count(channel_name) == 1)
        {
            throw InvalidMapKey::NameAlreadyExists(channel_name);
        }
        this->checkChannelNameIsNotReserved(channel_name);
        std::shared_ptr<VoxelGrid> voxel_grid  = Constructor::create(parser, this->grid_properties);
        voxel_grid->addSeenData(this->data_seen);
        this->channels.insert( {channel_name, voxel_grid} );
        this->updateMinAndMaxDist();
    }


    /// @brief Gets a constant reference to the VoxelGrid data channel.
    /// @param name Name of the channel in the channel dictionary to retrieve.
    /// @returns Read-only reference to the requested VoxelGrid data channel.
    /// @throws InvalidMapKey If a channel with that name does not exist.
    std::shared_ptr<const forge_scan::data::VoxelGrid> getChannelView(const std::string& name) const
    {
        return this->getChannelRef(name);
    }


    /// @brief Gets a reference to the VoxelGrid data channel.
    /// @param name Name of the channel in the channel dictionary to retrieve.
    /// @returns Reference to the requested VoxelGrid data channel.
    /// @throws InvalidMapKey If a channel with that name does not exist.
    std::shared_ptr<forge_scan::data::VoxelGrid> getChannelRef(const std::string& name) const
    {
        for (auto iter = this->channels.begin(); iter != this->channels.end(); ++iter)
        {
            if (iter->first == name)
            {
                return iter->second;
            }
        }
        throw InvalidMapKey::NonexistantValue(name);
    }


    /// @brief Removes a VoxelGrid data channel.
    /// @param name Name of the channel in the channel dictionary to remove.
    /// @returns True if successful. False if that channel did not exits or if it is owned by a
    ///          Metric or a Policy.
    bool removeChannel(const std::string& name)
    {
        for (auto iter = this->channels.begin(); iter != this->channels.end(); ++iter)
        {
            if (iter->first == name && iter->second.use_count() <= 1)
            {
                this->channels.erase(iter);
                return true;
            }
        }
        return false;
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Shared, constant `Grid::Properties` used by all VoxelGrids.
    const std::shared_ptr<const Grid::Properties> grid_properties;

    static const std::string parse_name;


private:
    // ***************************************************************************************** //
    // *                                PRIVATE CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param grid_properties Shared, constant pointer to the `Grid::Properties` to use.
    ///                        These `Grid::Properties` are utilized by all VoxelGrids.
    explicit Reconstruction(const std::shared_ptr<const Grid::Properties>& grid_properties)
        : grid_properties(grid_properties),
          data_seen(std::make_shared<std::vector<bool>>(this->grid_properties->getNumVoxels(), false)),
          ray_trace(std::make_shared<Trace>())
    {

    }


    /// @brief Saves `Grid::Properties` into the HDF5 file as attributes and calls the save method
    ///        for each VoxelGrid in the channel dictionary.
    /// @param h5_file An opened HDF5 file to write data into.
    void save(HighFive::File& h5_file)
    {
        auto g_reconstruction = h5_file.createGroup(FS_HDF5_RECONSTRUCTION_GROUP);

        g_reconstruction.createAttribute("VoxelGrid Resolution", this->grid_properties->resolution);
        g_reconstruction.createAttribute("VoxelGrid Dimensions", this->grid_properties->dimensions);
        g_reconstruction.createAttribute("VoxelGrid Size",       this->grid_properties->size);

        for (const auto& item : this->channels)
        {
            HighFive::Group g_channel = g_reconstruction.createGroup(item.first);
            item.second->save(g_channel, item.second->getTypeName());
        }
    }


    /// @brief Adds each VoxelGrid's data to the XDMF file provided by the Manager.
    /// @param file An opened file stream.
    /// @param hdf5_fname File name (not the full path) of the HDF5 file that this XDMF relates to.
    void addToXDMF(std::ofstream& file, const std::string& hdf5_fname) const
    {
        for (const auto& item : this->channels) {
            const std::string attr_name = item.second->getTypeName();
            item.second->addToXDMF(file, hdf5_fname, item.first, attr_name);
        }
    }


    /// @brief Helper to update the minimum min and maximum max values each time the channel dict
    ///        changes.
    void updateMinAndMaxDist() {
        this->min_dist_min = 0;
        this->max_dist_max = 0;

        for (const auto& item : this->channels)
        {
            // Record the most negative distance as min and most positive distance as max.
            this->min_dist_min = std::min(this->min_dist_min, item.second->dist_min);
            this->max_dist_max = std::max(this->max_dist_max, item.second->dist_max);
        }

        // Verify that we never have a case where the min is greater than the max.
        // This should never happen but it would be a critical error if it did.
        assert(this->max_dist_max >= this->min_dist_min &&
               "Reconstruction maximum dist max was less than the minimum dist min. "
               "This should not be possible.");
    }


    /// @brief Used by the Metric class to request a new VoxelGrid channel be added.
    /// @param channel Shared pointer to the VoxelGrid channel to add.
    /// @param metric_name Name of the Metric class adding the channel.
    /// @throws InvalidMapKey If `metic_name` is empty.
    /// @throws InvalidMapKey If a channel of the same name already exists.
    void metricAddChannel(std::shared_ptr<data::VoxelGrid> channel,
                          const std::string& metric_name)
    {
        if (metric_name.empty())
        {
            throw InvalidMapKey::NoNameProvided();
        }

        const std::string channel_name = FS_METRIC_CHANNEL_PREFIX + metric_name;
        if (this->channels.count(channel_name) == 1)
        {
            throw InvalidMapKey::NameAlreadyExists(channel_name);
        }

        this->channels.insert( {channel_name, channel} );
        this->updateMinAndMaxDist();
    }


    /// @brief Used by the Policy class to request a new VoxelGrid channel be added.
    /// @param channel Shared pointer to the VoxelGrid channel to add.
    /// @param metric_name Name of the Policy class adding the channel.
    /// @throws ReservedMapKey If `policy_name` is empty.
    /// @throws ReservedMapKey If a channel of the same name already exists.
    void policyAddChannel(std::shared_ptr<data::VoxelGrid> channel,
                          const std::string& policy_name)
    {
        if (policy_name.empty())
        {
            throw ReservedMapKey("Policy must provide a name to add a new channel.");
        }

        const std::string channel_name = FS_POLICY_CHANNEL_PREFIX + policy_name;
        if (this->channels.count(channel_name) == 1)
        {
            throw ReservedMapKey("A channel named \"" + channel_name +"\" already exists.");
        }

        this->channels.insert( {channel_name, channel} );
        this->updateMinAndMaxDist();
    }



    // ***************************************************************************************** //
    // *                                STATIC CLASS METHODS                                   * //
    // ***************************************************************************************** //


    /// @brief Verifies that the requested channel name does not begin with a reserved prefix.
    /// @param name A channel name to check.
    /// @throws ReservedMapKey if the channel name is reserved for Metrics or Policies.
    static void checkChannelNameIsNotReserved(const std::string& name)
    {
        if (channelNameIsForPolicies(name))
        {
            throw ReservedMapKey("A name beginning with \"Policy\" is reserved "
                                "and may not be created or destroyed." );
        }
        else if (channelNameIsForMetrics(name))
        {
            throw ReservedMapKey("A name beginning with \"Metric\" is reserved "
                                "and may not be created or destroyed." );
        }
    }


    /// @brief Returns true if the name is reserved for Metrics only.
    /// @param name The requested channel name to be checked.
    static bool channelNameIsForMetrics(const std::string& name)
    {
        return utilities::strings::hasPrefix(name,
                                             FS_METRIC_CHANNEL_PREFIX,
                                             FS_METRIC_CHANNEL_PREFIX_C_STR_LEN);
    }


    /// @brief Returns true if the name is reserved for Policies only.
    /// @param name The requested channel name to be checked.
    static bool channelNameIsForPolicies(const std::string& name)
    {
        return utilities::strings::hasPrefix(name,
                                             FS_POLICY_CHANNEL_PREFIX,
                                             FS_POLICY_CHANNEL_PREFIX_C_STR_LEN);
    }



    // ***************************************************************************************** //
    // *                                PRIVATE CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Dictionary mapping the name of a channel to its VoxelGrid.
    std::map<std::string, std::shared_ptr<VoxelGrid>> channels;

    /// @brief Record the extreme min and max dist for across all VoxelGrids. This constrains ray
    ///        trace calculations to only the regions which VoxelGrids will actually update.
    float min_dist_min = 0, max_dist_max = 0;

    /// @brief Container of the same shape as `VoxelGrid::data` in any derived grid, but this stores
    ///        a boolean flag for if a voxel was intersected by the postive region of a ray at least.
    ///        It may be used by some grids in creating an occupancy data vector or in the update method.
    std::shared_ptr<std::vector<bool>> data_seen;
    
    /// @brief Stores an ray trace used for performing updates on each VoxelGrid.
    std::shared_ptr<Trace> ray_trace;
};


/// @brief ArgParser key for the name of the Data Channel to be add.
const std::string Reconstruction::parse_name = "--name";


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTION_RECONSTRUCTION_HPP
