#include <iostream>
#include <fstream>
#include <iomanip>

/// Eigen is used when saving/loading from HDF5 files.
#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>
#include <highfive/H5PropertyList.hpp>

#include "ForgeScan/Metrics/sensor_record.h"


namespace ForgeScan {
namespace Metrics   {


void SensorRecord::save(const std::filesystem::path& fname, const bool add_to_grid) const {
    if ( !fname.has_filename() )
        throw std::invalid_argument("[SensorRecord::save] Invalid file name! Could not identify filename.");
    try
    {
        auto mode = add_to_grid ? HighFive::File::ReadWrite : HighFive::File::Truncate;
        auto file = HighFive::File(fname.string() + ".h5", mode);

        auto metrics_group = file.createGroup("Metrics");
        auto sensor_record_group = metrics_group.createGroup("SensorRecord");

        /// TODO: It would be helpful to track creation order for the groups; however this is not supported in
        ///       the current HighFive version I am using. I'll update this later. For now any program that iterates
        ///       must a file written by this function must ensure it is accessing the groups in the order itself.
        /// See:
        ///     https://github.com/BlueBrain/HighFive/blob/64b7ad9a9cca566de1103ac74abab710b6f928cb/CHANGELOG.md?plain=1#L20-L23

        int i = 0;
        for (const auto& sensor : record) {
            auto group = sensor_record_group.createGroup(std::to_string(i));
            group.createAttribute("total_updates",  sensor.total_updates);
            group.createAttribute("total_views",    sensor.total_views);
            group.createAttribute("total_first",    sensor.total_first);
            group.createAttribute("percent_hit",    sensor.percent_hit);
            group.createAttribute("percent_viewed", sensor.percent_viewed);
            group.createAttribute("max_variance_update", sensor.max_variance_update);
            group.createAttribute("pose", sensor.pose.matrix());
            ++i;
        }
    }
    catch (const HighFive::Exception& err)
    {
        std::cerr << err.what() << std::endl;
    }
}


} // namespace Metrics
} // namespace ForgeScan
