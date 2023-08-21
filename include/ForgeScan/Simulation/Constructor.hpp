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
/// @param args Arguments to pass into the Primitive's create functions.
/// @return A pointer to the requested implementation of the Primitive class.
/// @throws std::invalid_argument if the primitive type is not recognized.
std::shared_ptr<Primitive> primitive_constructor(const utilities::ArgParser& parser)
{
    std::string shape = parser.getCmdOption("--shape");
    utilities::strings::toLower(shape);

    if (shape == "box")
    {
        return Box::create(parser);
    }
    else if (shape == "sphere")
    {
        return Sphere::create(parser);
    }
    else
    {
        throw std::invalid_argument("The primitive shape type of \"" + shape + "\" was not recognized.");
    }
}


} // namespace primitives
} // namespace forge_scan


#endif // FORGE_SCAN_PRIMITIVE_CONSTRUCTOR_HPP
