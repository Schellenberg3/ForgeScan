#ifndef FORGE_SCAN_UTILITIES_MEMORY_USE_HPP
#define FORGE_SCAN_UTILITIES_MEMORY_USE_HPP

#include <cstddef>
#include <vector>

// Define conversion utility for bits_to_megabytes (1e-6) for this header file only.
#define MB_PER_BYTE 0.000001


namespace forge_scan {
namespace utilities {
namespace memory_use {


/// @brief Finds the memory usage of a vector.
/// @param vec Vector of any type.
/// @return The number of bytes used by the size of the vector.
template<typename T>
inline size_t vector_size(const typename std::vector<T>& vec)
{
    return sizeof(T) * vec.size();
}


/// @brief Finds the memory usage of a vector.
/// @param vec Vector of any type.
/// @return The number of bytes used by the capacity of the vector.
template<typename T>
inline size_t vector_capacity(const typename std::vector<T>& vec)
{
    return sizeof(T) * vec.capacity();
}


/// @brief Converts bytes to megabytes.
/// @param bytes Number in bytes.
/// @return Number in megabytes.
inline float byte_to_megabytes(const size_t& bytes)
{
    return static_cast<float>(bytes) * MB_PER_BYTE;
}


} // namespace memory_use
} // namespace utilities
} // namespace forge_scan


#ifdef MB_PER_BYTE
    #undef MB_PER_BYTE
#endif

#endif // FORGE_SCAN_UTILITIES_MEMORY_USE_HPP
