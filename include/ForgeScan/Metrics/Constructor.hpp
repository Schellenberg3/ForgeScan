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
/// @throws ConstructorError if the Metric type is not recognized.
/// @throws std::invalid_argument for OccupancyConfusion type. This type must be added manually.
inline std::shared_ptr<Metric> metric_constructor(const utilities::ArgParser& args,
                                                  std::shared_ptr<data::Reconstruction> /*reconstruction*/)
{
    std::string metric_type = args.get(Metric::parse_type);
    utilities::strings::toLower(metric_type);

    if (metric_type == "occupancyconfusion")
    {
        throw std::invalid_argument("The Metric type of OccupancyConfusion requires additional parameters that this method cannot provide.");
    }
    else
    {
    }

    throw ConstructorError::UnkownType(metric_type, "Metric");
}


} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_CONSTRUCTOR_HPP
