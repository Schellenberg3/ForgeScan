#ifndef FORGE_SCAN_PRIMITIVE_CONSTRUCTOR_HPP
#define FORGE_SCAN_PRIMITIVE_CONSTRUCTOR_HPP

#include <memory>
#include <algorithm>

#include "ForgeScan/Simulation/Box.hpp"
#include "ForgeScan/Simulation/Sphere.hpp"

#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace simulation {


/// @brief Constructor for shared Primitive pointers based on ArgParser inputs.
struct Constructor
{
    /// @brief Factory function to create different Primitive types.
    /// @param parser Arguments to pass into the Primitive's create functions.
    /// @return A pointer to the requested implementation of the Primitive class.
    /// @throws ConstructorError if the Primitive type is not recognized.
    static std::shared_ptr<Primitive> create(const utilities::ArgParser& parser)
    {
        using namespace utilities::strings;
        std::string shape = parser.get(Primitive::parse_shape);

        if (iequals(shape, Box::type_name))
        {
            return Box::create(parser);
        }
        if (iequals(shape, Sphere::type_name))
        {
            return Sphere::create(parser);
        }

        throw ConstructorError::UnkownType(shape, Primitive::type_name);
    }


    /// @brief Returns a string help message for a constructing Primitive shapes.
    /// @param parser Arguments to pass determine which help information to print.
    static std::string help(const utilities::ArgParser& parser)
    {
        using namespace utilities::strings;
        std::string shape = parser.get("-h");

        if (iequals(shape, Box::type_name))
        {
            return Box::helpMessage();
        }
        if (iequals(shape, Sphere::type_name))
        {
            return Sphere::helpMessage();
        }
        std::stringstream ss;
        ss << Primitive::helpMessage() << "\nPossible shapes are: "
           << Box::type_name << ", "
           << Sphere::type_name;
        return ss.str();
    }
};


} // namespace primitives
} // namespace forge_scan


#endif // FORGE_SCAN_PRIMITIVE_CONSTRUCTOR_HPP
