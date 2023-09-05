#ifndef FORGE_SCAN_COMMON_DATA_VARIANTS_HPP
#define FORGE_SCAN_COMMON_DATA_VARIANTS_HPP

#include <cstdint>
#include <cstddef>
#include <string>
#include <variant>
#include <vector>
#include <map>


#include "ForgeScan/Utilities/Strings.hpp"


namespace forge_scan {


/// @brief Union of possible types of data that a VoxelGrid may hold.
typedef
std::variant<
    int8_t,
    int16_t,
    int32_t,
    int64_t,

    uint8_t,
    uint16_t,
    uint32_t,
    size_t,

    float,
    double
>
DataVariant;


/// @brief Union of std::vectors for the possible types of data, DataVariant, that a VoxelGrid may hold.
/// @note Each vector only holds one data type. This union interact with it without caring what the Data is until runtime.
typedef
std::variant<
    std::vector<int8_t>,
    std::vector<int16_t>,
    std::vector<int32_t>,
    std::vector<int64_t>,

    std::vector<uint8_t>,
    std::vector<uint16_t>,
    std::vector<uint32_t>,
    std::vector<size_t>,

    std::vector<float>,
    std::vector<double>
>
VectorVariant;


/// @brief Identification and type checking for the DataVariants a Grid has or may have.
/// @note Enumerations beginning with TYPE_* are used for checking types, not for assigning them.
enum DataType
    : uint8_t
{
    INT8_T  = 0b1000'0001,
    INT16_T = 0b1000'0010,
    INT32_T = 0b1000'0011,
    INT64_T = 0b1000'0100,

    UINT8_T      = 0b0100'0001,
    UINT16_T     = 0b0100'0010,
    UINT32_T     = 0b0100'0011,
    SIZE_T       = 0b0100'0100,

    FLOAT  = 0b0010'0001,
    DOUBLE = 0b0010'0010,

    TYPE_ANY              = 0b1111'0000,
    TYPE_INT              = 0b1100'0000,
    TYPE_SIGNED_INT       = 0b1000'0000,
    TYPE_UNSIGNED_INT     = 0b0100'0000,
    TYPE_FLOATING_POINT   = 0b0010'0000,
    TYPE_NOT_A_TYPE_CHECK = 0b0000'1111,
};


/// @brief Constant map of DataType enumerations to their std::string representation.
const std::map<DataType, std::string> DataTypeToString = {
    {DataType::INT8_T,  "INT8_T"},
    {DataType::INT16_T, "INT16_T"},
    {DataType::INT32_T, "INT32_T"},
    {DataType::INT64_T, "INT64_T"},

    {DataType::UINT8_T,  "UINT8_T"},
    {DataType::UINT16_T, "UINT16_T"},
    {DataType::UINT32_T, "UINT32_T"},
    {DataType::SIZE_T,   "SIZE_T"},

    {DataType::FLOAT,  "FLOAT"},
    {DataType::DOUBLE, "DOUBLE"},

    {DataType::TYPE_ANY,              "TYPE_ANY"},
    {DataType::TYPE_INT,              "TYPE_INT"},
    {DataType::TYPE_SIGNED_INT,       "TYPE_SIGNED_INT"},
    {DataType::TYPE_UNSIGNED_INT,     "TYPE_UNSIGNED_INT"},
    {DataType::TYPE_FLOATING_POINT,   "TYPE_FLOATING_POINT"},
    {DataType::TYPE_NOT_A_TYPE_CHECK, "TYPE_NOT_A_TYPE_CHECK"},
};


/// @brief Constant map of std::string representations to to their DataType enumerations.
/// @note Each enumeration has multiple keys based on upper/lower case and with/without the *_T suffix.
const std::map<std::string, DataType> StringToDataType = {
    {"INT8_T",   DataType::INT8_T},
    {"INT16_T",  DataType::INT16_T},
    {"INT32_T",  DataType::INT32_T},
    {"INT64_T",  DataType::INT64_T},
    {"INT8",     DataType::INT8_T},
    {"INT16",    DataType::INT16_T},
    {"INT32",    DataType::INT32_T},
    {"INT64",    DataType::INT64_T},

    {"UINT8_T",  DataType::UINT8_T},
    {"UINT16_T", DataType::UINT16_T},
    {"UINT32_T", DataType::UINT32_T},
    {"SIZE_T",   DataType::SIZE_T},
    {"UINT64_T", DataType::SIZE_T},
    {"UINT8",    DataType::UINT8_T},
    {"UINT16",   DataType::UINT16_T},
    {"UINT32",   DataType::UINT32_T},
    {"SIZE",     DataType::SIZE_T},
    {"UINT64",   DataType::SIZE_T},

    {"FLOAT",    DataType::FLOAT},
    {"DOUBLE",   DataType::DOUBLE},

    {"TYPE_ANY",              DataType::TYPE_ANY},
    {"TYPE_INT",              DataType::TYPE_INT},
    {"TYPE_SIGNED_INT",       DataType::TYPE_SIGNED_INT},
    {"TYPE_UNSIGNED_INT",     DataType::TYPE_UNSIGNED_INT},
    {"TYPE_FLOATING_POINT",   DataType::TYPE_FLOATING_POINT},
    {"TYPE_NOT_A_TYPE_CHECK", DataType::TYPE_NOT_A_TYPE_CHECK},
};


/// @brief Performs safe conversion between a string and a DataType enumeration
/// @param type_id_string Input string to the StringToDataType dict.
/// @param default_value Default value to return if the string was not found.
/// @return
inline DataType stringToDataType(const std::string& type_id_string, const DataType& default_value)
{
    std::string type_id_string_upper = type_id_string;
    utilities::strings::toUpper(type_id_string_upper);
    try
    {
        return StringToDataType.at(type_id_string_upper);
    }
    catch (const std::exception&)
    {
        return default_value;
    }
}


/// @brief Returns a string for an XDMF DataItem's NumberType field.
/// @param x The DataType to get a NumberType string for.
/// @throws invalid_argument if `x` is not recognized as a DataType.
inline std::string getNumberTypeXDMF(const DataType& x)
{
    // The types int8_t and uint8_t specifically correlate to the Char and UChar in XDMF to match with the precision value of "1"
    // There's poor documentation on this. The best reference I found on XDMF DataItem tags is:
    //   https://visit-sphinx-github-user-manual.readthedocs.io/en/task-allen-vtk9_master_ospray/data_into_visit/XdmfFormat.html#dataitem
    if (x == DataType::UINT8_T)
    {
        return "UChar";
    }
    else if (x == DataType::INT8_T)
    {
        return "Char";
    }
    else if (x & DataType::TYPE_UNSIGNED_INT)
    {
        return "UInt";
    }
    else if (x & DataType::TYPE_SIGNED_INT)
    {
        return "Int";
    }
    else if (x & DataType::TYPE_FLOATING_POINT)
    {
        return "Float";
    }
    else
    {
        throw std::invalid_argument("Unrecognized data type_id: " + std::to_string(x) +
                                    ". Cannot return the XDMF number type.");
    }
}


/// @brief Returns a string for an XDMF DataItem's Precision field.
/// @param x The DataType to get a NumberType string for.
/// @throws invalid_argument if `x` is not recognized as a DataType.
inline std::string getNumberPrecisionXDMF(const DataType& x)
{
    if (x == DataType::UINT8_T ||
        x == DataType::INT8_T)
    {
        return "1";
    }
    else if (x == DataType::UINT16_T ||
             x == DataType::INT16_T)
    {
        return "2";
    }
    else if (x == DataType::UINT32_T ||
             x == DataType::INT32_T  ||
             x == DataType::FLOAT)
    {
        return "4";
    }
    else if (x == DataType::SIZE_T  ||
             x == DataType::INT64_T ||
             x == DataType::DOUBLE)
    {
        return "8";
    }
    else
    {
        throw std::invalid_argument("Unrecognized data type_id: " + std::to_string(x) +
                                    ". Cannot return the XDMF number precision.");
    }
}


/// @brief Enumeration for occupancy-style Grids.
enum VoxelOccupancy
    : uint8_t
{
    UNKNOWN  = 0b0000'0000,
    FREE     = 0b0100'0001,
    OCCUPIED = 0b1000'0001,
    CLIPPED  = 0b1000'0010,

    TYPE_FREE     = 0b0100'0000,
    TYPE_OCCUPIED = 0b1000'0000,
};


} // forge_scan


#endif // FORGE_SCAN_COMMON_DATA_VARIANTS_HPP
