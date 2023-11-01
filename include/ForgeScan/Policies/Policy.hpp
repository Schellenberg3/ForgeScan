#ifndef FORGE_SCAN_POLICIES_POLICY_HPP
#define FORGE_SCAN_POLICIES_POLICY_HPP

#include <memory>
#include <list>

#define H5_USE_EIGEN 1
#include <highfive/H5Easy.hpp>

#include "ForgeScan/Common/Exceptions.hpp"
#include "ForgeScan/Data/Reconstruction.hpp"


namespace forge_scan {

    // Forward definition to allow friend access.
    class Manager;

} // forge_scan


namespace forge_scan {
namespace policies {


/// @brief Base implementation for a Policy that suggests new views for Reconstruction.
class Policy
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @details Requires access to save data.
    friend class forge_scan::Manager;


public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    virtual ~Policy() { }

    /// @brief Queries the Policy to get its suggested view.
    /// @return Gets the best view suggested by the Policy.
    Extrinsic getView()
    {
        if (views.empty())
        {
            this->generate();
        }
        assert(views.empty() == false &&
               "The current Policy was unable to suggest views, even after calling generate.");
        return views.front();
    }


    /// @return Number of views a consumer has accepted from this policy.
    size_t numAccepted() const
    {
        return this->accepted_views.size();
    }


    /// @return Number of views a consumer has rejected from this policy.
    size_t numRejected() const
    {
        return this->rejected_views.size();
    }


    /// @return Help message for constructing a Policy with ArgParser.
    static std::string helpMessage()
    {
        return "A Policy generates views which may be added to a Reconstruction. Some policies follow a geometric "
               "algorithm while others use a data-driven approach."
               "\nA Policy may be created with the following arguments:"
               "\n\t" + Policy::help_string +
               "\n\nFor details on specific Policy options, enter \"-h <policy type>\".";
    }


    // ***************************************************************************************** //
    // *                           PURE VIRTUAL PUBLIC CLASS METHODS                           * //
    // ***************************************************************************************** //


    /// @brief Returns the type name of the derived Policy class.
    virtual const std::string& getTypeName() const = 0;


    /// @brief Prints information about the Policy to the output stream.
    /// @param out Output stream to write to.
    virtual void print(std::ostream& out) const = 0;


    /// @brief Returns true if the Policy believed the scan is complete.
    virtual bool isComplete() const = 0;


    /// @brief Remove previously generated list of suggested views and generates a new batch.
    virtual void generate() = 0;


    static const int default_n_views;

    static const float default_seed;

    static const std::string parse_set_active, parse_type, parse_n_views, parse_seed;

    static const std::string help_string;

    static const std::string type_name;

protected:
    /* ***************************************************************************************** */
    /*                                 PROTECTED CLASS METHODS                                   */
    /* ***************************************************************************************** */


    /// @brief Protected constructor for derived classes only.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests
    ///                       new views for.
    Policy(const std::shared_ptr<data::Reconstruction>& reconstruction)
        : reconstruction(reconstruction)
    {

    }


    /// @brief Helper for derived classes to add a channel durring set-up.
    /// @param channel Channel (created in the derived class) to add.
    /// @param policy_name Name of the derived Policy class.
    void addChannel(const std::shared_ptr<data::VoxelGrid>& channel,
                    const std::string& policy_name)
    {
        return this->reconstruction->policyAddChannel(channel, policy_name);
    }


    /// @brief Accepts the view returned by this->getView and removed it from the list.
    /// @param count Total number of views accepted/rejected from all policies.
    ///              This is just used as a simple unique, ordered identifier.
    /// @returns True if the suggested view was accepted. False if there was no view to accept.
    bool acceptView(const size_t& count = 0)
    {
        if ( !this->views.empty() )
        {
            this->accepted_views.push_back( {count, this->views.front()} );
            views.pop_front();
            return true;
        }
        return false;
    }


    /// @brief Rejects the view returned by this->getView and removed it from the list.
    /// @param count Total number of views accepted/rejected from all policies.
    ///              This is just used as a simple unique, ordered identifier.
    /// @returns True if the suggested view was accepted. False if there was no view to accept.
    bool rejectView(const size_t& count = 0)
    {
        if ( !this->views.empty() )
        {
            this->rejected_views.push_back( {count, this->views.front()} );
            this->views.pop_front();
            return true;
        }
        return false;
    }


    static void saveViews(H5Easy::File& file, const std::string& policy_name,
                          std::list<std::pair<size_t, forge_scan::Extrinsic>> id_and_view,
                          const std::string& label)
    {
        const std::string hdf5_data_root = "/" FS_HDF5_POLICY_GROUP "/" + policy_name + "/" + label;
        std::stringstream ss;
        for (const auto& list_item : id_and_view)
        {
            ss << hdf5_data_root << "/" << list_item.first;
            H5Easy::dump(file, ss.str(), list_item.second.matrix());
            ss.str(std::string());
        }
    }


    /// @brief Saves the rejected views, and their order identifier, to the HDF5 file.
    /// @param file File to write to.
    /// @param policy_name Name of the derived Policy class.
    void saveRejectedViews(H5Easy::File& file, const std::string& policy_name) const
    {
        saveViews(file, policy_name, this->rejected_views, "rejected");
    }

    /// @brief Saves the accepted views, and their order identifier, to the HDF5 file.
    /// @param file File to write to.
    /// @param policy_name Name of the derived Policy class.
    void saveAcceptedViews(H5Easy::File& file, const std::string& policy_name) const
    {
        saveViews(file, policy_name, this->accepted_views, "accepted");
    }



    // ***************************************************************************************** //
    // *                              PROTECTED VIRTUAL METHODS                                * //
    // ***************************************************************************************** //


    /// @brief Runs when a Policy is added to the Manager.
    ///        This give the Policy the option to add data channels it might need.
    virtual void setup()
    {

    }


    /// @brief Save the Policy information into an HDF5 file.
    /// @param file The HDF5 file to write to.
    /// @param g_policy The specific group to write Policy information in.
    virtual void save(H5Easy::File& file, HighFive::Group& g_policy) const = 0;



    // ***************************************************************************************** //
    // *                               PROTECTED STATIC METHODS                                * //
    // ***************************************************************************************** //


    /// @param policy_name Name of the policy.
    /// @return Path in an HDF5 file  where the Policy should read/write information from.
    static std::string getDatasetPathHDF5(const std::string& policy_name)
    {
        return "/" FS_HDF5_POLICY_GROUP "/" + policy_name + "/data";
    }



    // ***************************************************************************************** //
    // *                               PROTECTED CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief Storage for the suggested views the Policy has generated.
    std::list<Extrinsic> views;

    /// @brief Storage for the views that were a Policy and accepted by the consumer.
    std::list<std::pair<size_t, Extrinsic>> accepted_views;


    /// @brief Storage for the views that were generated by the Policy but rejected by the consumer.
    std::list<std::pair<size_t, Extrinsic>> rejected_views;

    /// @brief Reference to the Reconstruction class. Some Policies use this to add a specific data
    ///        channel which they require.
    const std::shared_ptr<data::Reconstruction> reconstruction;
};


/// @brief Prints info about the Primitive to the output stream.
/// @param out Output stream to write to.
/// @param policy The Policy to write to the output stream.
/// @return Reference to the output stream.
std::ostream& operator<<(std::ostream &out, const Policy& policy)
{
    policy.print(out);
    return out;
}


/// @brief String for the class name.
const std::string Policy::type_name = "Policy";

/// @brief Default number of views for a Policy to collect.
const int Policy::default_n_views = 10;

/// @brief Default RNG seed for a Policy to use.
const float Policy::default_seed = -1;

/// @brief ArgParser key for the flag to set a new Policy as the active one.
const std::string Policy::parse_set_active = "--set-active";

/// @brief ArgParser key for the type of Policy to add.
const std::string Policy::parse_type = "--type";

/// @brief ArgParser key for number of views a Policy should collect. Policies may generate more
///        views than this but some require an expected number up-front or use this to judge if
///        they have reached completion.
const std::string Policy::parse_n_views = "--n-views";

/// @brief ArgParser key for the seed to use to initialize the Policy's RNG.
///        Negative values indicate the program should use a random RNG seed.
const std::string Policy::parse_seed = "--seed";

/// @brief String explaining what arguments a generic Policy class accepts.
const std::string Policy::help_string =
    Policy::parse_type + " <policy type> [policy-specific options]";


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_POLICY_HPP
