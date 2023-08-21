#ifndef FORGE_SCAN_POLICIES_CONSTRUCTOR_HPP
#define FORGE_SCAN_POLICIES_CONSTRUCTOR_HPP

#include <memory>
#include <algorithm>

#include "ForgeScan/Policies/Policy.hpp"

#include "ForgeScan/Policies/Simple/RandomSphere.hpp"

#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace policies {


/// @brief Factory function to create different Policy types.
/// @param args Arguments to pass into the Policy's create functions.
/// @param reconstruction Shared, constant pointer to the Reconstruction passed to the Policy's create function.
/// @return A pointer to the requested implementation of the Policy class.
/// @throws std::invalid_argument if the Policy type is not recognized.
inline std::shared_ptr<Policy> policy_constructor(const utilities::ArgParser& parser,
                                                  std::shared_ptr<const data::Reconstruction> reconstruction)
{
    std::string policy_type = parser.getCmdOption("--policy-type");
    utilities::strings::toLower(policy_type);

    if (policy_type == "randomsphere")
    {
        return RandomSphere::create(reconstruction, parser);
    }
    else
    {
        throw std::invalid_argument("The Policy type of \"" + policy_type + "\" was not recognized.");
    }
}


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_CONSTRUCTOR_HPP
