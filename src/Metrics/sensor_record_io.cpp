#include <iostream>
#include <fstream>
#include <iomanip>

/// Eigen is used when saving/loading from HDF5 files.
#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>
#include <highfive/H5PropertyList.hpp>
#include <highfive/H5Version.hpp>

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

        HighFive::GroupCreateProps properties{};
        #if HIGHFIVE_VERSION_MAJOR >= 2 && HIGHFIVE_VERSION_MINOR >= 7
        properties.add(HighFive::LinkCreationOrder(HighFive::CreationOrder::Tracked | HighFive::CreationOrder::Indexed));
        #endif
        auto sensor_record_group = metrics_group.createGroup("SensorRecord", properties);

        int i = 0;
        for (const auto& sensor : *this) {
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
