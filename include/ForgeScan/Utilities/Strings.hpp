#ifndef FORGE_SCAN_UTILITIES_STRINGS_HPP
#define FORGE_SCAN_UTILITIES_STRINGS_HPP

#include <algorithm>
#include <cctype>
#include <string>


namespace forge_scan {
namespace utilities {
namespace strings {


/// @brief Transforms, in-place, a string to all lower cases.
/// @param data Input string.
inline void toLower(std::string& data)
{
    std::transform(data.begin(), data.end(), data.begin(), [](unsigned char c){ return std::tolower(c); });
}


/// @brief Transforms, in-place, a string to all upper cases.
/// @param data Input string.
inline void toUpper(std::string& data)
{
    std::transform(data.begin(), data.end(), data.begin(), [](unsigned char c){ return std::toupper(c); });
}


/// @brief Checks if the string has a specific prefix.
/// @param str String to check.
/// @param prefix Prefix to look for as an array of characters.
/// @param n Length of the character array.
/// @return True if the string has the prefix.
inline bool hasPrefix(const std::string& str, const char* prefix, const size_t& n)
{
    return str.rfind(prefix, n) == 0;
}


/// @brief Checks if the string has a specific prefix.
/// @param str String to check.
/// @param prefix Prefix to look for.
/// @return True if the string has the prefix.
inline bool hasPrefix(const std::string& str, const std::string& prefix)
{
    return str.rfind(prefix, 0) == 0;
}


} // namespace strings
} // namespace utilities
} // namespace forge_scan


#endif // FORGE_SCAN_UTILITIES_STRINGS_HPP
