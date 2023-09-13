#ifndef FORGE_SCAN_UTILITIES_FILES_HPP
#define FORGE_SCAN_UTILITIES_FILES_HPP

#include <ctime>
#include <sstream>
#include <iomanip>


namespace forge_scan {
namespace utilities {


/// @brief Generates a default time-stamped filename whe one is not provided.
/// @param prefix    A generic name to place before the timestamp.
/// @param extension A file extension to place after the time stamp.
/// @param timestamp If true, will add a timestamp.
/// @returns A string to be used as a file name with the structure `[prefix]-[timestamp][extension]`.
inline std::string getDefaultFilename(const std::string& prefix = "",
                                      const std::string& extension = "",
                                      const bool& timestamp = true)
{
    std::stringstream ss;
    if(!prefix.empty())
    {
        ss << prefix << "-";
    }

    if (timestamp)
    {
        auto t  =  std::time(nullptr);
        auto tm = *std::localtime(&t);
        ss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    }

    if(!extension.empty())
    {
        ss << extension;
    }

    return ss.str();
}


/// @brief Verifies the file path has the specified file name and extension.
/// @param fpath Path to check and possible modify.
/// @param extension Extension to check for.
/// @param default_fname File name to use if none is provided.
/// @param timestamp_default If true, will add a timestamp if the default file name is used.
inline void checkPathHasFileNameAndExtension(std::filesystem::path& fpath, const std::string& extension = "",
                                             const std::string& default_fname = "", const bool& timestamp_default = true)
{
    if (!fpath.has_filename())
    {
        fpath.replace_filename(utilities::getDefaultFilename(default_fname, extension, timestamp_default));
    }
    if ( !(fpath.extension() == extension) )
    {
        fpath.replace_extension(extension);
    }
}


} // namespace utilities
} // namespace forge_scan


#endif // FORGE_SCAN_UTILITIES_FILES_H