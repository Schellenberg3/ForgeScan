#ifndef FORGE_SCAN_RECONSTRUCTION_GRID_CONSTRUCTOR_HPP
#define FORGE_SCAN_RECONSTRUCTION_GRID_CONSTRUCTOR_HPP

#include <memory>

#include "ForgeScan/Data/VoxelGrids/Binary.hpp"
#include "ForgeScan/Data/VoxelGrids/BinaryTSDF.hpp"
#include "ForgeScan/Data/VoxelGrids/CountViews.hpp"
#include "ForgeScan/Data/VoxelGrids/CountUpdates.hpp"
#include "ForgeScan/Data/VoxelGrids/Probability.hpp"
#include "ForgeScan/Data/VoxelGrids/TSDF.hpp"

#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {
namespace data {


/// @brief Constructor for shared VoxelGrid pointers based on ArgParser inputs.
struct Constructor
{
    /// @brief Factory function to create different VoxelGrid types.
    /// @param parser Arguments to pass into the VoxelGrid's create functions.
    /// @param properties A shared, constant reference to the `Grid::Properties` for the Reconstruction.
    /// @return A pointer to the requested implementation of the VoxelGrid class.
    /// @throws ConstructorError if the VoxelGrid type is not recognized.
    static std::shared_ptr<VoxelGrid> create(const utilities::ArgParser& parser,
                                             std::shared_ptr<const Grid::Properties> properties)
    {
        using namespace utilities::strings;
        std::string grid_type = parser.get(VoxelGrid::parse_type);

        if (iequals(grid_type, Binary::type_name))
        {
            return Binary::create(properties, parser);
        }
        if (iequals(grid_type, BinaryTSDF::type_name))
        {
            return BinaryTSDF::create(properties, parser);
        }
        if (iequals(grid_type, CountViews::type_name))
        {
            return CountViews::create(properties, parser);
        }
        if (iequals(grid_type, Probability::type_name))
        {
            return Probability::create(properties, parser);
        }
        if (iequals(grid_type, TSDF::type_name))
        {
            return TSDF::create(properties, parser);
        }
        if (iequals(grid_type, CountUpdates::type_name))
        {
            return CountUpdates::create(properties, parser);
        }

        throw ConstructorError::UnkownType(grid_type, VoxelGrid::type_name);
    }


    /// @brief Returns a string help message for constructing a VoxelGrid.
    /// @param parser Arguments to pass determine which help information to print.
    static std::string help(const utilities::ArgParser& parser)
    {
        using namespace utilities::strings;
        std::string grid_type = parser.get("-h");

        if (iequals(grid_type, Binary::type_name))
        {
            return Binary::helpMessage();
        }
        if (iequals(grid_type, BinaryTSDF::type_name))
        {
            return BinaryTSDF::helpMessage();
        }
        if (iequals(grid_type, CountViews::type_name))
        {
            return CountViews::helpMessage();
        }
        if (iequals(grid_type, Probability::type_name))
        {
            return Probability::helpMessage();
        }
        if (iequals(grid_type, TSDF::type_name))
        {
            return TSDF::helpMessage();
        }
        if (iequals(grid_type, CountUpdates::type_name))
        {
            return CountUpdates::helpMessage();
        }
        std::stringstream ss;
        ss << VoxelGrid::helpMessage() << "\nPossible data VoxelGrids are: "
           << Binary::type_name      << ", "
           << BinaryTSDF::type_name  << ", "
           << Probability::type_name << ", "
           << TSDF::type_name        << ", "
           << CountUpdates::type_name;
        return ss.str();
    }
};



} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTION_GRID_CONSTRUCTOR_HPP
