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


/// @brief Trims space characters, in place, from the beginning of the string.
/// @param data Input string.
/// @details Adapted from https://stackoverflow.com/questions/216823/how-to-trim-an-stdstring/217605#217605
inline void ltrim(std::string &data)
{
    data.erase(data.begin(),
               std::find_if(data.begin(), data.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}


/// @brief Trims space characters, in place, from the end of the string.
/// @param data Input string.
/// @details Adapted from https://stackoverflow.com/questions/216823/how-to-trim-an-stdstring/217605#217605
inline void rtrim(std::string &data)
{
    data.erase(std::find_if(data.rbegin(), data.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
               data.end());
}


/// @brief Trims space characters, in place, from the beginning and end of the string.
/// @param data Input string.
/// @details Adapted from https://stackoverflow.com/questions/216823/how-to-trim-an-stdstring/217605#217605
inline void trim(std::string &data)
{
    rtrim(data);
    ltrim(data);
}


/// @brief Checks if the string has interesting contents.
/// @param data Input string.
/// @return False if the string is empty or only contains space characters
inline bool has_contents(const std::string &data)
{
    return !data.empty() && (data.find_first_not_of(' ') != std::string::npos);
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


/// @brief Case-insensitive character comparison.
/// @param a First character.
/// @param b Second character.
/// @return True if they are the same character in either lower or upper case.
bool ichar_equals(const char& a, const char& b)
{
    return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b));
}


/// @brief Case-insensitive string comparison.
/// @param a First string.
/// @param b Second string.
/// @return True if the strings have the same contents, regardless of the capitalization of the contents.
bool iequals(const std::string& a, const std::string& b)
{
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), ichar_equals);
}


} // namespace strings
} // namespace utilities
} // namespace forge_scan


#endif // FORGE_SCAN_UTILITIES_STRINGS_HPP
