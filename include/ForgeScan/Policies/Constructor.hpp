#ifndef FORGE_SCAN_POLICIES_CONSTRUCTOR_HPP
#define FORGE_SCAN_POLICIES_CONSTRUCTOR_HPP

#include <memory>
#include <algorithm>

#include "ForgeScan/Policies/Policy.hpp"

#include "ForgeScan/Policies/Simple/Sphere.hpp"
#include "ForgeScan/Policies/Simple/Axis.hpp"

#include "ForgeScan/Policies/Heuristic/Occplane.hpp"

#include "ForgeScan/Policies/Precomputed/Normal.hpp"

#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace policies {


/// @brief Constructor for shared Policy pointers based on ArgParser inputs.
struct Constructor
{
    /// @brief Factory function to create different Policy types.
    /// @param parser Arguments to pass into the Policy's create functions.
    /// @param reconstruction Shared, constant pointer to the Reconstruction passed to the Policy's create function.
    /// @return A pointer to the requested implementation of the Policy class.
    /// @throws ConstructorError if the Policy type is not recognized.
    static std::shared_ptr<Policy> create(const utilities::ArgParser& parser,
                                          const std::shared_ptr<data::Reconstruction>& reconstruction)
    {
        using namespace utilities::strings;
        std::string policy_type = parser.get(Policy::parse_type);

        if (iequals(policy_type, Sphere::type_name))
        {
            return Sphere::create(reconstruction, parser);
        }
        if (iequals(policy_type, Axis::type_name))
        {
            return Axis::create(reconstruction, parser);
        }
        if (iequals(policy_type, OccplaneInfo::type_name))
        {
            return Occplane::create(reconstruction, parser);
        }
        if (iequals(policy_type, NormalInfo::type_name))
        {
            return Normal::create(reconstruction, parser);
        }

        throw ConstructorError::UnkownType(policy_type, Policy::type_name);
    }


    /// @brief Returns a string help message for constructing a Policy.
    /// @param parser Arguments to pass determine which help information to print.
    static std::string help(const utilities::ArgParser& parser)
    {
        using namespace utilities::strings;
        std::string policy_type = parser.get("-h");

        if (iequals(policy_type, Sphere::type_name))
        {
            return Sphere::helpMessage();
        }
        if (iequals(policy_type, Axis::type_name))
        {
            return Axis::helpMessage();
        }
        if (iequals(policy_type, Occplane::type_name))
        {
            /// TODO: Return and implement this.
            return "TODO: Write occplane help.";
        }
        if (iequals(policy_type, NormalInfo::type_name))
        {
            /// TODO: Return and implement this.
            return "TODO: Write normal help.";
        }
        std::stringstream ss;
        ss << Policy::helpMessage() << "\nPossible Policies are: "
           << Sphere::type_name << ", "
           << Axis::type_name;
        return ss.str();
    }
};


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_CONSTRUCTOR_HPP
