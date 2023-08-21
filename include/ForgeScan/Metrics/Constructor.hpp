#ifndef FORGE_SCAN_METRICS_CONSTRUCTOR_HPP
#define FORGE_SCAN_METRICS_CONSTRUCTOR_HPP

#include <memory>

#include "ForgeScan/Metrics/OccupancyConfusion.hpp"
#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace metrics {


/// @brief Factory function to create different Metric types.
/// @param args Arguments to pass into the Metric's create functions.
/// @return A pointer to the requested implementation of the Metric class.
/// @throws std::invalid_argument if the Metric type is not recognized.
inline std::shared_ptr<Metric> metric_constructor(const utilities::ArgParser& args,
                                                  std::shared_ptr<data::Reconstruction> reconstruction)
{
    std::string metric_type = args.getCmdOption("--metric");
    utilities::strings::toLower(metric_type);

    if (metric_type == "occupancyconfusion")
    {
        throw std::invalid_argument("The Metric type of OccupancyConfusion requires additional parameters that this method cannot provide.");
    }
    else
    {
    }
    throw std::invalid_argument("The Metric type of \"" + metric_type + "\" was not recognized.");
}


} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_CONSTRUCTOR_HPP
