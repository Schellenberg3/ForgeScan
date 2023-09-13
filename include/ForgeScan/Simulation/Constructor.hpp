#ifndef FORGE_SCAN_PRIMITIVE_CONSTRUCTOR_HPP
#define FORGE_SCAN_PRIMITIVE_CONSTRUCTOR_HPP

#include <memory>
#include <algorithm>

#include "ForgeScan/Simulation/Box.hpp"
#include "ForgeScan/Simulation/Sphere.hpp"

#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace simulation {


/// @brief Factory function to create different Primitive types.
/// @param parser Arguments to pass into the Primitive's create functions.
/// @return A pointer to the requested implementation of the Primitive class.
/// @throws ConstructorError if the Primitive type is not recognized.
std::shared_ptr<Primitive> primitive_constructor(const utilities::ArgParser& parser)
{
    std::string shape = parser.get(Primitive::parse_shape);
    utilities::strings::toLower(shape);

    if (shape == "box")
    {
        return Box::create(parser);
    }
    else if (shape == "sphere")
    {
        return Sphere::create(parser);
    }

    throw ConstructorError::UnkownType(shape, "Primitive");
}


} // namespace primitives
} // namespace forge_scan


#endif // FORGE_SCAN_PRIMITIVE_CONSTRUCTOR_HPP
