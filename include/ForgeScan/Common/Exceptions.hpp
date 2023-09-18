#ifndef FORGE_SCAN_COMMON_EXCEPTIONS_H
#define FORGE_SCAN_COMMON_EXCEPTIONS_H


#include <exception>
#include <sstream>

#include "ForgeScan/Common/Types.hpp"


namespace forge_scan {


/// @brief Base class for all ForgeScan exceptions.
struct Exception : public std::runtime_error
{
    /// @brief Constructs a ForgeScan Exception.
    /// @param what Exception message.
    explicit Exception(const std::string& what)
        : std::runtime_error(what)
    {

    }
};


/// @brief Exception class for invalid access of maps used in Manager, Reconstruction,
///        Scene, and potentially others.
/// @details All the maps used by these classes link a string key to a shared pointer
///          and may be accessed at runtime. This exception class describes the errors
///          that may be encountered then.
struct InvalidMapKey : public Exception
{
    /// @brief Constructs an InvalidMapKey exception.
    /// @param what Exception message.
    explicit InvalidMapKey(const std::string& what)
        : Exception(what)
    {

    }


    /// @brief Constructor for access when a key was not provided (or the kay was an empty string).
    /// @return InvalidMapKey with message.
    static inline InvalidMapKey NoNameProvided()
    {
        return InvalidMapKey("No name was provided for the map.");
    }


    /// @brief Constructor for access when a key does not have an associated value.
    /// @param name Key name.
    /// @return InvalidMapKey with message.
    static inline InvalidMapKey NonexistantValue(const std::string& name = "")
    {
        return InvalidMapKey("No value exists in the map for the specified key: " + name );
    }


    /// @brief Constructor for access when a key already exists with an associated value.
    /// @param name Key name.
    /// @return InvalidMapKey with message.
    static inline InvalidMapKey NameAlreadyExists(const std::string& name = "")
    {
        return InvalidMapKey("The name \"" + name + "\" already exists in the map.");
    }
};



/// @brief Exception class for InvalidMapKey when the key is otherwise acceptable but
///        is reserved for another use.
struct ReservedMapKey : public InvalidMapKey
{
    /// @brief Constructs a ReservedMapKey exception.
    /// @param what Exception message.
    explicit ReservedMapKey(const std::string& what)
        : InvalidMapKey(what)
    {

    }
};


/// @brief Exception class for Constructor factories when parsing ArgParser to select which of a
///        base class's derived classes to create. Also used in class constructors in cases where
///        some options might be mutually exclusive. 
struct ConstructorError : public Exception
{
    /// @brief Constructs an ConstructorError exception.
    /// @param what Exception message.
    explicit ConstructorError(const std::string& what)
        : Exception(what)
    {

    }


    /// @brief Constructs message to specify what name was not found as a valid derived class.
    /// @param derived_type Name of the requested derived type.
    /// @param base_type Name of the base class.
    /// @return ConstructorError with message.
    static ConstructorError UnkownType(const std::string& derived_type, const std::string& base_type)
    {
        return ConstructorError("The type \"" + derived_type + "\" in not recognized as a valid type of " + base_type + ".");
    }


    /// @brief Constructs message to specify that selected options are mutually exclusive.
    /// @param type Name of the requested class.
    /// @param a First mutually exclusive option
    /// @param b Second mutually exclusive option.
    /// @return ConstructorError with message.
    static ConstructorError MutuallyExclusive(const std::string& type, const std::string& a, const std::string& b)
    {
        return ConstructorError("In \"" + type + "\" the options \"" + a + "\" and \"" + b + "\" are mutually exclusive.");
    }
};


/// @brief Exception class for issues with the `Grid::Properties` class.
struct GridPropertyError : public Exception
{
    /// @brief Constructs a GridPropertyError exception.
    /// @param what Exception message.
    explicit GridPropertyError(const std::string& what)
        : Exception(what)
    {

    }


    /// @brief Constructor for non-matching `Grid::Properties` between a source and destination Grid.
    /// @param x Identifier or name for one grid.
    /// @param y Identifier or name for the other grid.
    /// @return GridPropertyError with message.
    static inline GridPropertyError PropertiesDoNotMatch(const std::string& x,
                                                         const std::string& y)
    {
        return GridPropertyError("Grid Properties from " + x + " do not match those of " + y);
    }


    /// @brief Constructor for non-matching GridSize and data vector length
    /// @param size Dimensions of the Grid.
    /// @param len  Length of the data vector.
    /// @return GridPropertyError with message.
    static inline GridPropertyError DataVectorDoesNotMatch(const GridSize& size, const size_t& len)
    {
        std::ostringstream oss;
        oss << "Grid Properties with dimension (" << size.transpose()
            << ") do not match with the data vector of length " << len << ".";
        return GridPropertyError(oss.str());
    }
};


/// @brief Exception class for if Index exceeds a `Grid::Properties` GridSize.
struct VoxelOutOfRange : public GridPropertyError
{
    /// @brief Constructs a VoxelOutOfRange exception.
    /// @param what Exception message.
    explicit VoxelOutOfRange(const std::string& what)
        : GridPropertyError(what)
    {

    }

    /// @brief Constructs a VoxelOutOfRange exception.
    /// @param size Grid size.
    /// @param idx Requested index.
    VoxelOutOfRange(const GridSize& size, const Index& idx)
        : GridPropertyError(message(size, idx))
    {

    }

private:
    static std::string message(const GridSize& size, const Index& idx)
    {
        std::ostringstream oss;
        oss << "Grid Properties with dimension (" << size.transpose()
            << ") do not contain voxel (" << idx.transpose() << ").";
        return oss.str();
    }
};


/// @brief Exception class for failed downcasts from VoxelGrid pointer to a derived class.
struct BadVoxelGridDownCast : public Exception
{
    /// @brief Constructs a GridPropertyError exception.
    /// @param derived_type_name Name of the derived class.
    explicit BadVoxelGridDownCast(const std::string& derived_type_name)
        : Exception("Downcast from VoxelGrid to " + derived_type_name + " failed.")
    {

    }
};


/// @brief Exception class for DataVariant and DataType issues.
struct DataVariantError : public Exception
{
    /// @brief Constructs a DataVariantError exception.
    /// @param what Exception message.
    explicit DataVariantError(const std::string& what)
        : Exception(what)
    {

    }


    /// @brief Constructor for if the DataType enumeration is not recognized.
    /// @param value Unrecognized enumeration value.
    /// @return DataVariantError with message.
    /// @note This variation of DataVariantError should not occur. But if it does, then something has
    ///       gone terribly wrong. Especially if this is thrown outside of the VoxelGrid constructor.
    static DataVariantError UnrecognizedEnumeration(const int& value)
    {
        return DataVariantError("DataType enumeration value of \"" + std::to_string(value) +
                                "\" was not recognized.");
    }


    /// @brief Constructor for invalid DataType enumerations reserved for type checking.
    /// @param data_type The type checking data type attempted.
    /// @return DataVariantError with message.
    static DataVariantError AttemptedUseOfTypeCheckingDataType(const std::string& data_type)
    {
        return DataVariantError("Cannot use type checking DataType of \"" + data_type +
                                " to construct a DataVariant.");
    }


    /// @brief Constructor for a VoxelGrid constructor to catch an unsupported DataType.
    /// @param attempted_type The data type attempted.
    /// @param supported_type The data types supported.
    /// @return DataVariantError with message.
    static DataVariantError VoxelGridDoesNotSupport(const std::string& attempted_type,
                                                    const std::string& supported_type)
    {
        return DataVariantError("VoxelGrid does not support \"" + attempted_type +
                                "\" types, it accepts data types of " + supported_type + ".");
    }
};


} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_EXCEPTIONS_H
