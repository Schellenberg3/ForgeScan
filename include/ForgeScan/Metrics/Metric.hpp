#ifndef FORGE_SCAN_METRICS_METRIC_HPP
#define FORGE_SCAN_METRICS_METRIC_HPP

/// Using H5-easy because of Eigen errors.
#include <highfive/H5Easy.hpp>

#include "ForgeScan/Common/Exceptions.hpp"
#include "ForgeScan/Common/Types.hpp"
#include "ForgeScan/Data/Reconstruction.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"


namespace forge_scan {

    // Forward definition to allow friend access.
    class Manager;

} // forge_scan


namespace forge_scan {
namespace metrics {


/// @brief Base implementation for a Metric that collects data about a Reconstruction.
class Metric
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @details Required to call pre/post update and save methods.
    friend class forge_scan::Manager;


public:
    virtual ~Metric() { }

    /// @return Help message for constructing a Metric with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: Metric help message";
    }

    static const std::string parse_type;

    static const std::string type_name;

    /// @brief Name for this Metric in the map. If only one metric of this type should exist then
    ///        the type name can be used.
    const std::string map_name;

protected:
    // ***************************************************************************************** //
    // *                                PROTECTED CLASS METHODS                                * //
    // ***************************************************************************************** //


    /// @brief Protected constructor for derived classes only.
    /// @param reconstruction Shared pointer to the `data::Reconstruction` that the Metric observes.
    /// @param map_name String key for the `Manager`'s map of names to metrics.
    Metric(const std::shared_ptr<data::Reconstruction>& reconstruction,
          const std::string map_name)
        : map_name(map_name),
          reconstruction(reconstruction)          
    {

    }


    /// @brief Helper for derived classes to add a channel durring set-up.
    /// @param channel Channel (created in the derived class) to add.
    /// @param metric_name Name of the derived Metric class.
    void addChannel(const std::shared_ptr<data::VoxelGrid>& channel,
                    const std::string& metric_name)
    {
        return this->reconstruction->metricAddChannel(channel, metric_name);
    }


    // ***************************************************************************************** //
    // *                               PROTECTED VIRTUAL METHODS                               * //
    // ***************************************************************************************** //


    /// @brief Runs when a Metric is added to the Manager.
    ///        This give the Metric the option to add data channels it might need.
    virtual void setup()
    {

    }


    /// @brief Runs before the Manager processes an update.
    /// @param sensed The sensed points passed to `Manager::reconstructionUpdate`.
    /// @param extr   The reference frame and common origin for the `sensed` points passed to
    ///               `Manager::reconstructionUpdate`.
    /// @param update_count A count of how many times the Reconstruction has been updated with
    ///                     new data. This is tracked by the Manager class.
    virtual void preUpdate([[maybe_unused]] const PointMatrix& sensed, [[maybe_unused]] const Extrinsic& extr,
                           [[maybe_unused]] const size_t& update_count)
    {

    }


    /// @brief Runs after the Manager has processed an update.
    /// @param update_count A count of how many times the `data::Reconstruction` has been updated with
    ///                     new data. This is tracked by the Manager class.
    virtual void postUpdate([[maybe_unused]] const size_t& update_count)
    {

    }



    // ***************************************************************************************** //
    // *                            PROTECTED PURE VIRTUAL METHODS                             * //
    // ***************************************************************************************** //


    /// @brief Saves the data stored by the Metric.
    /// @param file HDF5 file to save to.
    virtual void save(HighFive::File& file) const = 0;


    /// @brief Returns the type name of the derived Policy class.
    virtual const std::string& getTypeName() const = 0;



    // ***************************************************************************************** //
    // *                               PROTECTED STATIC METHODS                                * //
    // ***************************************************************************************** //


    /// @brief Creates the dataset path for the derived Metric class.
    /// @param metric_name String name for the derived Metric. (see `Metric::getTypeName`)
    /// @return Path in an HDF5 file where the Metric should read/write information.
    static std::string getDatasetPathHDF5(const std::string& metric_name)
    {
        return "/" FS_HDF5_METRIC_GROUP "/" + metric_name + "/data";
    }



    // ***************************************************************************************** //
    // *                               PROTECTED CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief Reference to the Reconstruction class. Some Metrics use this to add a specific data
    ///        channel which they require.
    const std::shared_ptr<data::Reconstruction> reconstruction{nullptr};
};


/// @brief String for the class name.
const std::string Metric::type_name = "Metric";

/// @brief ArgParser key for the type of Metric to add.
const std::string Metric::parse_type = "--type";


} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_METRIC_HPP
