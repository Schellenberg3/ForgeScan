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


/// @brief Linear interpolation between `a` and `b`.
/// @param a First value.
/// @param b Second value.
/// @param t Proportion between `a` and `b`.
/// @note  Assumes `0<=t<=1` but does not check this.
/// @return `a+t(b−a)`
template <typename T>
inline T lerp(const T& a, const T& b, const T& t)
{
    return a * (1.0 - t) + (b * t);
}


/// @brief Turns the log-odds probability into a probability value
/// @param x Log-odds value.
/// @return Probability for the log odds.
template <typename T>
inline T probability(const T& x)
{
    return std::exp(x) / (1 + std::exp(x));
}


/// @brief Turns the probability value into its log-odds value.
/// @param p First value.
/// @return Log odds for the probability.
/// @warning This does not check if 0 <= p <= 1.
template <typename T>
inline T log_odds(const T& p)
{
    return std::log(p) - std::log(1 - p);
}


} // namespace math
} // namespace utilities
} // namespace forge_scan


#endif // FORGE_SCAN_UTILITIES_MATH_HPP
