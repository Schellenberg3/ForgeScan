#ifndef FORGE_SCAN_COMMON_GRID_HPP
#define FORGE_SCAN_COMMON_GRID_HPP

#include <memory>
#include <iostream>

#include "ForgeScan/Common/Types.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Common/VoxelData.hpp"


namespace forge_scan {


/// @brief Base interface for Grid of uniformly-sized voxels.
/// @details Derived classes are responsible for providing a vector for the voxel data.
///          This vector stores data continuously in memory, and is expected to
///          increment fastest in X, then Y, then Z.
/// @note  The `Grid` class and those derived from it may be treated as axis-aligned bounding box
///        (AABB). The AABB's shape is based on the Grid Properties: the lower bound is always at
///        (0, 0, 0) while the upper bound is at the Property's `size*resolution`, or `dimension`.
///        Any operations with 3D data assumes it is already transformed into a reference frame
///        placed at the lower bound of the AABB. Generally a higher-level class, like Manager,
///        will handle this.
struct Grid
{
    /// @brief Defines the properties for voxel size and spacing and provides methods to convert
    ///        between voxel indicies, and continuous coordinates, and vector positions.
    /// @note  See the note on `forge_scan::Grid` for details on using 3D data with this class.
    struct Properties
    {
        /// @brief Constructor based on resolution and Grid size, dimensions is set implicitly
        /// @param resolution Edge length of each voxel in world units. Default 0.02.
        /// @param size       Number of voxels in the Grid in each direction. Default (101, 101, 101)
        /// @note Ensures there is a minimum GridSize of (1, 1, 1).
        /// @note Ensures the resolution is positive.
        Properties(const float& resolution = 0.02,
                   const GridSize& size = GridSize(101, 101, 101))
            : resolution(resolution),
              size(size)
        {
            this->setDimensions();
        }


        /// @brief Constructs the Grid Properties based on the provided Parser.
        /// @param parser ArgParser with arguments to construct Grid Properties from.
        /// @note Ensures there is a minimum GridSize of (1, 1, 1).
        /// @note Ensures the resolution is positive.
        Properties(const utilities::ArgParser& parser)
            : resolution(parser.get<float>(Properties::parse_resolution, Properties::default_resolution)),
              size(std::max(parser.get<int>(Properties::parse_nx, Properties::default_size), 1),
                   std::max(parser.get<int>(Properties::parse_ny, Properties::default_size), 1),
                   std::max(parser.get<int>(Properties::parse_nz, Properties::default_size), 1))
        {
            this->setDimensions();
        }


        Properties(const Properties& other)
            : resolution(other.resolution),
              size(other.size)
        {
            this->setDimensions();
        }


        /// @brief Creates a shared pointer to a constant Grid Properties.
        /// @param resolution Edge length of each voxel in world units.
        /// @param size       Number of voxels in the Grid in each direction. Default (101, 101, 101)
        /// @note Ensures there is a minimum GridSize of (1, 1, 1).
        /// @note Ensures the resolution is positive.
        static std::shared_ptr<const Properties> createConst(const float& resolution = 0.02,
                                                             const GridSize& size = GridSize(101, 101, 101))
        {
            return std::shared_ptr<Properties>(new Properties(resolution, size));
        }


        /// @brief Creates a shared pointer to a constant Grid Properties.
        /// @param parser ArgParser with arguments to construct Grid Properties from.
        /// @note Ensures there is a minimum GridSize of (1, 1, 1).
        /// @note Ensures the resolution is positive.
        static std::shared_ptr<const Properties> createConst(const utilities::ArgParser& parser)
        {
            return std::shared_ptr<Properties>(new Properties(parser));
        }


        /// @brief Creates a shared pointer to a constant Grid Properties.
        /// @param other The Grid Properties to copy.
        /// @note Ensures there is a minimum GridSize of (1, 1, 1).
        /// @note Ensures the resolution is positive.
        static std::shared_ptr<const Properties> createConst(const Properties& other)
        {
            return std::shared_ptr<Properties>(new Properties(other));
        }


        /// @brief Interactively creates Grid Properties from a users input to stdin.
        /// @return Shared, constant pointer to Grid Properties.
        static std::shared_ptr<const Properties> createConstInteractive()
        {
            utilities::ArgParser parser;
            while (true)
            {
                parser.getInput("Please specify the Grid Properties to use [-h for help]:");
                if (parser[0] != "-h")
                {
                    std::shared_ptr<const Grid::Properties> properties = Properties::createConst(parser);
                    return properties;
                }
                std::cout << Properties::helpMessage() << "\n" << std::endl;
            }
        }


        /// @return Help message for constructing Grid Properties with ArgParser.
        static std::string helpMessage()
        {
            return "A Forge Scan Grid Properties object may be created with the following arguments:"
                   "\n\t" + Properties::help_string +
                   "\nIf the optional arguments are not provided, the default values are:"
                   "\n\t" + Properties::default_arguments;
        }


        /// @brief Compares two Grid Properties to verify that all values are equal.
        /// @param other The Grid Properties to compare against this.
        /// @return True if all values are equal.
        bool isEqual(const Properties& other) const
        {
            return (this->resolution         == other.resolution        )       &&
                   (this->size.array()       == other.size.array()      ).all() &&
                   (this->dimensions.array() == other.dimensions.array()).all() &&
                   (this->p2i_scale.array()  == other.p2i_scale.array() ).all();
        }


        /// @brief Compares two Grid Properties to verify that all values are equal.
        /// @param other Shared pointer to the Grid Properties to compare against this.
        /// @return True if all values are equal.
        bool isEqual(const std::shared_ptr<const Properties>& other) const
        {
            return this->isEqual(*other);
        }


        /// @brief Updates the dimensions (and point-to-index scale) to fit the size and resolution.
        /// @note Also ensures that the resolution is positive.
        /// @note Also ensures each dimension has a size of at least 1.
        void setDimensions()
        {
            this->checkMinimumGridSize();
            this->resolution = std::abs(this->resolution);
            this->dimensions = (size.cast<float>().array() - 1) * this->resolution;
            this->p2i_scale  = (size.cast<float>().array() - 1) / dimensions.array();
        }


        /// @brief Checks that the voxel Index is valid for the shape of the Grid.
        /// @param voxel Index for the desired voxel.
        /// @return True if valid. False otherwise.
        bool indexIsValid(const Index& voxel) const
        {
            return ( voxel.array() < this->size.array() ).all();
        }


        /// @brief Finds the vector Index for the provided voxel location.
        ///        Does not verify if this voxel is in the Grid.
        /// @param voxel Index for the desired voxel.
        /// @note Without checking out of bounds vector lookups may result in undefined behaviour.
        size_t operator[](const Index& voxel) const
        {
            return this->indexToVector(voxel);
        }


        /// @brief Finds the vector Index for the provided voxel location.
        ///        Verifies that this voxel is in the Grid.
        /// @param voxel Index for the desired voxel.
        /// @throws VoxelOutOfRange if the requested Index exceeds the Grid's size in any dimension.
        size_t at(const Index& voxel) const
        {
            return this->indexToVectorThrowOutOfRange(voxel);
        }

        /// @brief Finds the vector Index for the provided point location.
        ///        Verifies that this voxel is in the Grid.
        /// @param point Location, relative to the grid's reference frame, retrieve a voxel vector index for.
        /// @throws VoxelOutOfRange if the requested Index exceeds the Grid's size in any dimension.
        size_t at(const Point& point) const
        {
            return this->indexToVectorThrowOutOfRange(this->pointToIndex(point));
        }


        /// @brief Calculates to Index that the Point falls into within the Grid.
        /// @param input Cartesian position of the Point, relative to the Grid origin.
        /// @return Grid Index that the Point would be in.
        /// @note The input MUST be transformed to the Grid's coordinate system for valid results.
        /// @note Use `valid` to check if the returned index is within the Grid.
        Index pointToIndex(const Point& input) const
        {
            return (input.array() * this->p2i_scale.array()).round().cast<size_t>();
        }


        /// @brief Returns the center location of the Grid.
        Point getCenter() const
        {
            return 0.5 * this->dimensions.array();
        }


        /// @brief Returns the total number of voxels in the Grid.
        size_t getNumVoxels() const
        {
            return this->size.prod();
        }

        /// @brief Resolution of the voxels in world dimensions.
        /// @note  Value must be positive.
        float resolution;

        /// @brief Number of voxels in the X, Y, and Z direction.
        /// @note  Each direction must have a minimum of 1 voxel.
        GridSize size;

        /// @brief Size of the Grid in world units in the X, Y and Z dimensions.
        ///        This forms the upper bound of an implicit AABB .
        /// @note  It is set by the setDimensions method.
        Eigen::Vector3f dimensions;

        /// @brief Scaling factor for converting from points to indicies.
        /// @note  It is set by the setDimensions method.
        Eigen::Vector3f p2i_scale;

        static const std::string parse_nx, parse_ny, parse_nz;

        static const std::string parse_resolution;

        static const float default_resolution;
        static const size_t default_size;

        static const std::string help_string, default_arguments;

    private:
        /// @brief Ensures there is at least one voxel in each direction.
        void checkMinimumGridSize()
        {
            this->size = this->size.unaryExpr([](size_t x){ return x == 0 ? 1 : x;  });
        }


        /// @brief Retrieves the vector Index for the given X, Y, Z Index in the Grid.
        /// @param voxel Index for the desired voxel.
        /// @return Vector position for the desired voxel.
        /// @note This does not check that the input voxel's Index is valid.
        size_t indexToVector(const Index& voxel) const
        {
            return voxel[0] + (voxel[1] * this->size[0]) + (voxel[2] * this->size[0] * this->size[1]);
        }


        /// @brief Retrieves the vector Index for the given X, Y, Z Index in the Grid.
        /// @param voxel Index for the desired voxel.
        /// @return Vector position for the desired voxel.
        /// @throws VoxelOutOfRange if the requested voxel Index exceed the Grid's size in any dimension.
        size_t indexToVectorThrowOutOfRange(const Index& voxel) const
        {
            if (this->indexIsValid(voxel))
            {
                return indexToVector(voxel);
            }
            throw VoxelOutOfRange(this->size, voxel);
        }
    };


    virtual ~Grid() { }


    /// @brief Returns the class type name for the Grid or `data::VoxelGrid`.
    virtual const std::string& getTypeName() const = 0;


    /// @brief Shared, constant Properties of the Grid.
    const std::shared_ptr<const Grid::Properties> properties;

    /// @brief Enumeration to identify the data type stored in this Grid.
    /// @details This is used by all Grid implementation when writing XDMF files (indicating
    ///          NumberType and Precision) It also identifies the DataVariant/VectorVariant
    ///          that a VoxelGrid has at runtime.
    const DataType type_id;


protected:
    /// @brief Protected constructor to enforce use of derived classes.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param type_id DataType enumeration to identify what the data type this Voxel
    ///                Grid's data vector holds.
    Grid(const std::shared_ptr<const Grid::Properties> properties, const DataType& type_id)
        : properties(properties),
          type_id(type_id)
    {

    }
};


/// @brief Writes the contents of the Grid Properties to the output stream.
/// @param out Output stream to write to.
/// @param properties Grid Properties to write out.
/// @return Reference to the output stream.
std::ostream& operator<<(std::ostream &out, const Grid::Properties& properties)
{
    out << "grid properties with size of (" << properties.size.transpose() <<
           ") voxels with resolution of " << properties.resolution <<
           " for a bounded area of (" << properties.dimensions.transpose() << ")";
    return out;
}


/// @brief ArgParser key for the number of voxel in X, Y and Z.
const std::string Grid::Properties::parse_nx = std::string("--nx"),
                  Grid::Properties::parse_ny = std::string("--ny"),
                  Grid::Properties::parse_nz = std::string("--nz");

/// @brief ArgParser key for the resolution of the voxels.
const std::string Grid::Properties::parse_resolution = std::string("--resolution");

/// @brief Default resolution value.
const float  Grid::Properties::default_resolution = 0.02;

/// @brief Default size value (for each dimension).
const size_t Grid::Properties::default_size       = 101;

/// @brief String explaining what arguments this class accepts.
const std::string Grid::Properties::help_string =
    "[" + Properties::parse_resolution + " <dimension of a voxel>]" +
    " [" + Properties::parse_nx + " <number voxel in X>]" +
    " [" + Properties::parse_ny + " <number voxel in Y>]" +
    " [" + Properties::parse_nz + " <number voxel in Z>]";

/// @brief String explaining what this class's default parsed values are.
const std::string Grid::Properties::default_arguments =
    Properties::parse_resolution + " " + std::to_string(Properties::default_resolution) +
    " " + Properties::parse_nx + " " + std::to_string(Properties::default_size) +
    " " + Properties::parse_ny + " " + std::to_string(Properties::default_size) +
    " " + Properties::parse_nz + " " + std::to_string(Properties::default_size);


} // forge_scan


#endif // FORGE_SCAN_COMMON_GRID_HPP
