#ifndef FORGE_SCAN_UTILITIES_ARG_PARSER_HPP
#define FORGE_SCAN_UTILITIES_ARG_PARSER_HPP

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>


namespace forge_scan {
namespace utilities {


/// @brief   A simple argument parsing tool.
/// @details Adapted from:
///              Parsing Command Line Arguments in C++?
///                  https://stackoverflow.com/questions/865668/#868894
///              Right way to split an std::string into a vector<string>
///                 https://stackoverflow.com/questions/5607650#5607650
class ArgParser
{
friend std::ostream& operator<< (std::ostream&, const ArgParser&);

public:
    /// @brief Constructs the argument parser from the argument count and argument array.
    /// @param argc Count of arguments in the argument vector.
    /// @param argv Argument array.
    ArgParser(const int &argc, const char **argv)
    {
        this->parse(argc, argv);
    }


    /// @brief Constructs the argument parser from the argument count and argument array.
    /// @param args A string of arguments deliminated by a space.
    ArgParser(const std::string& args)
    {
        this->parse(args);
    }


    /// @brief Constructs the argument parser from the argument count and argument array.
    /// @param args A string of arguments deliminated by a space.
    ArgParser(const char* args)
    {
        this->parse(std::string(args));
    }


    /// @brief Changes the parsed arguments of the ArgParser to a new string.
    /// @param args A string of arguments deliminated by a space.
    void setArgs(const std::string& args)
    {
        this->tokens.clear();
        this->parse(args);
    }


    /// @brief Gets the command in the X-th position.
    /// @param option Which position to get.
    /// @return A view of the command string if it exists. Empty string if the flag was not included.
    const std::string& operator[](const size_t& x) const
    {
        if (this->tokens.size() < x)
        {
            return ArgParser::empty_string;
        }
        return this->tokens[x];
    }


    /// @brief Checks of the option flag was passed and returns the associated value.
    /// @param option Option flag.
    /// @return A view of the command flag's value if it exists. Empty string if the flag was not included.
    const std::string& getCmdOption(const std::string &option) const
    {
        std::vector<std::string>::const_iterator itr;
        itr = std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end())
        {
            return *itr;
        }
        return ArgParser::empty_string;
    }


    /// @brief Checks of the option flag was passed and returns the associated value.
    /// @tparam T Type to cast to.
    /// @param option Option flag.
    /// @return Value associated with the flag.
    /// @throws std::invalid_argument if the option flag was not found.
    template <typename T>
    T getCmdOption(const std::string &option) const
    {
        const std::string& opt = this->getCmdOption(option);
        if (opt.empty())
        {
            throw std::invalid_argument("Cannot find option  \"" + option + "\". No default was provided.");
        }
        std::stringstream ss(opt);
        T result;
        ss >> result;
        return result;
    }


    /// @brief Checks of the option flag was passed and returns the associated value.
    /// @tparam T Type to cast to.
    /// @param option Option flag.
    /// @param default_val Default value if that option flag was not found.
    /// @return Value associated with the flag or the default value.
    template <typename T>
    T getCmdOption(const std::string &option, const T& default_val) const
    {
        const std::string& opt = this->getCmdOption(option);
        if (opt.empty())
        {
            return default_val;
        }
        std::stringstream ss(opt);
        T result;
        ss >> result;
        return result;
    }


    /// @brief Checks if the specific option was included in the command line input.
    /// @param option Option to search check for.
    /// @return True if the option exists. False else.
    bool cmdOptionExists(const std::string &option) const
    {
        return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
    }


private:
    /// @brief Parses the inputs into the ArgParser's tokens
    /// @param argc Count of arguments in the argument vector.
    /// @param argv Argument array.
    void parse(const int &argc, const char **argv)
    {
        this->tokens.reserve(argc - 1);
        for (int i=1; i < argc; ++i)
        {
            this->tokens.push_back(std::string(argv[i]));
        }
    }


    /// @brief Parses the inputs into the ArgParser's tokens
    /// @param args A string of arguments deliminated by a space.
    void parse(const std::string& args)
    {
        static const char delim = ' ';
        std::stringstream ss(args);
        std::string s;
        while (std::getline(ss, s, delim))
        {
            this->tokens.push_back(s);
        }
    }

    /// @brief Default return of an empty string.
    const static std::string empty_string;

    /// @brief Vector of each input string
    std::vector <std::string> tokens;
};


const std::string ArgParser::empty_string = std::string("");


/// @brief Writes the contents of a ArgParser to the output stream.
/// @param out Output stream to write to.
/// @param parser ArgParser to write out.
/// @return Reference to the output stream.
std::ostream& operator<< (std::ostream &out, const ArgParser& parser)
{
    if (parser.tokens.size() == 0) return out;
    for (auto it = parser.tokens.begin(); ; )
    {
        out << *it;
        if (++it == parser.tokens.end())
        {
            return out;
        }
        else
        {
            out << " ";
        }
    }
}


} // namespace forge_scan
} // namespace forge_scan

#endif // FORGE_SCAN_UTILITIES_ARG_PARSER_HPP
