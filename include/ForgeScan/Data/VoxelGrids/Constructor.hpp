#ifndef FORGE_SCAN_RECONSTRUCTION_GRID_CONSTRUCTOR_HPP
#define FORGE_SCAN_RECONSTRUCTION_GRID_CONSTRUCTOR_HPP

#include <memory>

#include "ForgeScan/Data/VoxelGrids/Binary.hpp"
#include "ForgeScan/Data/VoxelGrids/BinaryTSDF.hpp"
#include "ForgeScan/Data/VoxelGrids/Probability.hpp"
#include "ForgeScan/Data/VoxelGrids/TSDF.hpp"
#include "ForgeScan/Data/VoxelGrids/UpdateCount.hpp"

#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace data {


/// @brief Factory function to create different Voxel Grid types.
/// @param args Arguments to pass into the VoxelGrid's create functions.
/// @param properties A shared, constant reference to the properties for this reconstruction.
/// @return A pointer to the requested implementation of the VoxelGrid class.
/// @throws std::invalid_argument if the VoxelGrid type is not recognized or if the DataType is not supported by this VoxelGrid
inline std::shared_ptr<VoxelGrid> grid_constructor(const utilities::ArgParser& args,
                                                   std::shared_ptr<const Grid::Properties> properties)
{
    std::string grid_type = args.get("--grid-type");
    utilities::strings::toLower(grid_type);

    if (grid_type == "binary")
    {
        return Binary::create(properties, args);
    }
    else if (grid_type == "binarytsdf")
    {
        return BinaryTSDF::create(properties, args);
    }
    else if (grid_type == "probability")
    {
        return Probability::create(properties, args);
    }
    else if (grid_type == "tsdf")
    {
        return TSDF::create(properties, args);
    }
    else if (grid_type == "updatecount")
    {
        return UpdateCount::create(properties, args);
    }
    else
    {
        throw std::invalid_argument("The Grid type of \"" + grid_type + "\" was not recognized.");
    }
}


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTION_GRID_CONSTRUCTOR_HPP
