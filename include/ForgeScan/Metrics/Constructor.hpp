#ifndef FORGE_SCAN_METRICS_CONSTRUCTOR_HPP
#define FORGE_SCAN_METRICS_CONSTRUCTOR_HPP

#include <memory>

#include "ForgeScan/Metrics/OccupancyConfusion.hpp"
#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace metrics {


/// @brief Constructor for shared Metrics pointers based on ArgParser inputs.
struct Constructor
{
    /// @brief Factory function to create different Metrics types.
    /// @param parser Arguments to pass into the Metrics's create functions.
    /// @param reconstruction Shared, constant pointer to the Reconstruction passed to the Metrics's create function.
    /// @return A pointer to the requested implementation of the Metrics class.
    /// @throws ConstructorError if the Metrics type is not recognized.
    static std::shared_ptr<Metric> create(const utilities::ArgParser& parser,
                                          [[maybe_unused]] const std::shared_ptr<const data::Reconstruction>& reconstruction)
    {
        using namespace utilities::strings;
        std::string metric_type = parser.get(Metric::parse_type);

        if (iequals(metric_type, OccupancyConfusion::type_name))
        {
            throw std::invalid_argument("The Metric type of " + OccupancyConfusion::type_name +
                                        "  requires ground-truth information that this method cannot parse.");
        }

        throw ConstructorError::UnkownType(metric_type, "Metric");
    }


    /// @brief Returns a string help message for constructing a Metric.
    /// @param parser Arguments to pass into the Metric's create functions.
    static std::string help(const utilities::ArgParser& parser)
    {
        using namespace utilities::strings;
        std::string metric_type = parser.get("-h");

        if (iequals(metric_type, OccupancyConfusion::type_name))
        {
            return OccupancyConfusion::helpMessage();
        }
        std::stringstream ss;
        ss << Metric::helpMessage() << "\nPossible Metrics are: "
           << OccupancyConfusion::type_name;
        return ss.str();
    }
};



} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_CONSTRUCTOR_HPP
