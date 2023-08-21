#ifndef FORGE_SCAN_UTILITIES_MATH_HPP
#define FORGE_SCAN_UTILITIES_MATH_HPP

#include <cmath>
#include <type_traits>


namespace forge_scan {
namespace utilities {
namespace math {


/// @brief Checks if the reference is strictly greater in magnitude than the test.
/// @param reference Reference value.
/// @param test      Value to test against the reference.
template <typename T>
inline bool is_greater_in_magnitude(const T& reference, const T& test)
{
    return std::abs(reference) > std::abs(test);
}


/// @brief Checks if the reference is strictly lesser in magnitude than the test.
/// @param reference Reference value.
/// @param test      Value to test against the reference.
template <typename T>
inline bool is_lesser_in_magnitude(const T& reference, const T& test)
{
    return std::abs(reference) < std::abs(test);
}


/// @brief Returns the value smallest in magnitude.
/// @param x First value.
/// @param y Second value
/// @return Smaller magnitude value.
template <typename T>
inline T smallest_magnitude(const T& x, const T& y)
{
    return std::abs(x) < std::abs(y) ? x : y;
}


/// @brief Returns the value greatest in magnitude.
/// @param x First value.
/// @param y Second value
/// @return Greater magnitude value.
template <typename T>
inline T greatest_magnitude(const T& x, const T& y)
{
    return std::abs(x) > std::abs(y) ? x : y;
}


} // namespace math
} // namespace utilities
} // namespace forge_scan


#endif // FORGE_SCAN_UTILITIES_MATH_HPP
