#ifndef FORGESCAN_UTILITIES_VECTOR_MEMORY_USE_H
#define FORGESCAN_UTILITIES_VECTOR_MEMORY_USE_H

#include <cstddef>
#include <vector>


namespace ForgeScan {
namespace Utilities {


// Conversion utility for bits_to_megabytes. 1e-6
#define MB_PER_BYTE 0.000001


/// @brief Templated function to find the size of a vector in bytes
/// @param vec Vector to find the size of.
/// @return Size in bytes of memory in the vector is currently using
template<typename T>
size_t const inline vector_size(const typename std::vector<T>& vec)
{
    return sizeof(T) * vec.size();
}


/// @brief Templated function to find the capacity of a vector in bytes
/// @param vec Vector to find the capacity of.
/// @return Size in bytes of memory in the vector has capacity for.
template<typename T>
size_t const inline vector_capacity(const typename std::vector<T>& vec)
{
    return sizeof(T) * vec.capacity();  // Note 
}


/// @brief Converts bytes to megabytes (MB).
/// @param bytes Number of bytes in memory.
/// @return Memory size converted to MB.
double const inline byte_to_megabytes(const size_t& bytes)
{
    return bytes * MB_PER_BYTE;
}


} // Utilities
} // ForgeScan

#endif // FORGESCAN_UTILITIES_VECTOR_MEMORY_USE_H
