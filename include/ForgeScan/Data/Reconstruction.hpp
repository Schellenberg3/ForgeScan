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
/// @note  The `Reconstruction` class implicitly uses the origin of the Grid Properties it was
///        instantiated with as its reference frame. It is required that any 3D data passed in has
///        was already transformed into this frame. Generally a Manager class handles this.
class Reconstruction
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @brief Required add/remove channels, call the save method, and call the update method.
    friend class forge_scan::Manager;

    /// @brief Required to add/remove Metric-specific channels.
    friend class metrics::Metric;

    /// @brief Required to add/remove Policy-specific channels.
    friend class policies::Policy;


public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Creates a shared pointer to a Reconstruction.
    /// @param grid_properties Shared, constant pointer to the Grid Properties to use.
    ///                        These Grid Properties are utilized by all Voxel Grids.
    /// @return Shared pointer to a Reconstruction.
    static std::shared_ptr<Reconstruction> create(std::shared_ptr<const Grid::Properties> grid_properties)
    {
        return std::shared_ptr<Reconstruction>(new Reconstruction(grid_properties));
    }


    /// @brief Updates each Voxel Grid based on the provided set of rays.
    /// @param sensed A set of measurements which act as the end points for a collection of rays.
    /// @param origin Common origin of the sensed points.
    /// @note Both `sensed` and `origin` are assumed to be in the Reconstruction's reference frame.
    void update(const PointMatrix& sensed_points, const Point& origin)
    {
        for (const auto& sensed : sensed_points.colwise())
        {
            get_ray_trace(this->ray_trace, sensed, origin, this->grid_properties,
                          this->min_dist_min, this->max_dist_max);

            for (const auto& item : this->channels)
            {
                item.second->update(this->ray_trace);
            }
        }
    }


    /// @brief Adds a Voxel Grid data channel to the Reconstruction.
    /// @param parser Arg Parser with arguments to construct a new Voxel Grid from.
    ///               See `forge_scan::data::Reconstruction::addChannel` for details.
    /// @throws `std::invalid_argument` If no name was provided for the channel.
    /// @throws `std::invalid_argument` If there is already a channel with that name.
    /// @throws `std::invalid_argument` If there is an issue with the Voxel Grid creation process.
    void addChannel(const utilities::ArgParser& parser)
    {
        std::string channel_name = parser.getCmdOption("--channel-name");
        if (channel_name.empty())
        {
            throw std::invalid_argument("Must provide a name for the new channel before it may be added to the reconstruction.");
        }
        else if (this->channels.count(channel_name) == 1)
        {
            throw std::invalid_argument("A channel named \"" + channel_name +"\" already exists.");
        }
        this->checkChannelNameIsNotReserved(channel_name);
        this->channels.insert( {channel_name, grid_constructor(parser, this->grid_properties)} );
        this->updateMinAndMaxDist();
    }


    /// @brief If a g a read-only reference to the Voxel Grid data channel.
    /// @param name Name of the channel in the channel dictionary to retrieve.
    /// @returns Read-only reference to the requested Voxel Grid data channel.
    /// @throws `std::runtime_error` If a channel with that name does not exist.
    std::shared_ptr<const forge_scan::data::VoxelGrid> getChannel(const std::string& name) const
    {
        for (auto iter = this->channels.begin(); iter != this->channels.end(); ++iter)
        {
            if (iter->first == name)
            {
                return iter->second;
            }
        }
        throw std::runtime_error("No channel with the name \"" + name + "\" exists.");
    }

    
    /// @brief Removes a Voxel Grid data channel.
    /// @param name Name of the channel in the channel dictionary to remove.
    /// @returns True if successful. False if that channel did not exits or if it is owned by a
    ///          Metric or a Policy.
    bool removeChannel(const std::string& name)
    {
        for (auto iter = this->channels.begin(); iter != this->channels.end(); ++iter)
        {
            if (iter->first == name && iter->second.unique())
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


    /// @brief Shared, constant Grid Properties used by all Voxel Grids. 
    const std::shared_ptr<const Grid::Properties> grid_properties;


private:
    // ***************************************************************************************** //
    // *                                PRIVATE CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param grid_properties Shared, constant pointer to the Grid Properties to use.
    ///                        These Grid Properties are utilized by all Voxel Grids.
    explicit Reconstruction(const std::shared_ptr<const Grid::Properties>& grid_properties)
        : grid_properties(grid_properties),
          ray_trace(std::make_shared<trace>())
    {

    }


    /// @brief Saves Grid Properties into the HDF5 file as attributes and calls the save method
    ///        for each Voxel Grid in the channel dictionary.
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


    /// @brief Adds each Voxel Grid's data to the XDMF file provided by the Manager. 
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


    /// @brief Used by the Metric class to request a new Voxel Grid channel be added. 
    /// @param channel Shared pointer to the Voxel Grid channel to add.
    /// @param metric_name Name of the Metric class adding the channel.
    /// @throws std::invalid_argument If `metic_name` is empty.
    /// @throws std::invalid_argument If a channel of the same name already exists.
    void metricAddChannel(std::shared_ptr<data::VoxelGrid> channel,
                          const std::string& metric_name)
    {
        if (metric_name.empty())
        {
            throw std::invalid_argument("Metric must provide a name to add a new channel.");
        }

        const std::string channel_name = FS_METRIC_CHANNEL_PREFIX + metric_name;
        if (this->channels.count(channel_name) == 1)
        {
            throw std::invalid_argument("A channel named \"" + channel_name +"\" already exists.");
        }

        this->channels.insert( {channel_name, channel} );
        this->updateMinAndMaxDist();
    }


    /// @brief Used by the Policy class to request a new Voxel Grid channel be added. 
    /// @param channel Shared pointer to the Voxel Grid channel to add.
    /// @param metric_name Name of the Policy class adding the channel.
    /// @throws std::invalid_argument If `policy_name` is empty.
    /// @throws std::invalid_argument If a channel of the same name already exists.
    void policyAddChannel(std::shared_ptr<data::VoxelGrid> channel,
                          const std::string& policy_name)
    {
        if (policy_name.empty())
        {
            throw std::invalid_argument("Policy must provide a name to add a new channel.");
        }

        const std::string channel_name = FS_POLICY_CHANNEL_PREFIX + policy_name;
        if (this->channels.count(channel_name) == 1)
        {
            throw std::invalid_argument("A channel named \"" + channel_name +"\" already exists.");
        }

        this->channels.insert( {channel_name, channel} );
        this->updateMinAndMaxDist();
    }



    // ***************************************************************************************** //
    // *                                STATIC CLASS METHODS                                   * //
    // ***************************************************************************************** //


    /// @brief Verifies that the requested channel name does not begin with a reserved prefix.
    /// @param name A channel name to check.
    /// @throws `std::invalid_argument` if the channel name is reserved for Metrics or Policies.
    static void checkChannelNameIsNotReserved(const std::string& name)
    {
        if (channelNameIsForPolicies(name))
        {
            throw std::invalid_argument("A name beginning with \"Policy\" is reserved "
                                        "and may not be created or destroyed." );
        }
        else if (channelNameIsForMetrics(name))
        {
            throw std::invalid_argument("A name beginning with \"Metric\" is reserved "
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


    /// @brief Dictionary mapping the name of a channel to its Voxel Grid.
    std::map<std::string, std::shared_ptr<VoxelGrid>> channels;

    /// @brief Record the extreme min and max dist for across all Voxel Grids. This constrains ray
    ///        trace calculations to only the regions which Voxel Grids will actually update.
    float min_dist_min = 0, max_dist_max = 0;

    /// @brief Stores an ray trace used for performing updates on each Voxel Grid. 
    std::shared_ptr<trace> ray_trace;
};


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTION_RECONSTRUCTION_HPP
