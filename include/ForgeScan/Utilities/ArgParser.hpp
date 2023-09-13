#ifndef FORGE_SCAN_UTILITIES_ARG_PARSER_HPP
#define FORGE_SCAN_UTILITIES_ARG_PARSER_HPP

#include <algorithm>
#include <limits>
#include <string>
#include <sstream>
#include <vector>

#include "Strings.hpp"


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

friend std::istream& operator>> (std::istream&, ArgParser&);

public:
    /// @brief Constructs an ArgParser ready to parse a string via `setArgs`.
    ArgParser()
    {
        this->clear();
    }


    /// @brief Constructs the ArgParser from the argument count and argument array.
    /// @param argc Count of arguments in the argument vector.
    /// @param argv Argument array.
    ArgParser(const int &argc, const char **argv)
    {
        this->parse(argc, argv);
    }


    /// @brief Constructs the ArgParser from the argument count and argument array.
    /// @param args A string of arguments deliminated by a space.
    ArgParser(const std::string& args)
    {
        this->parse(args);
    }


    /// @brief Constructs the ArgParser from the argument count and argument array.
    /// @param args A string of arguments deliminated by a space.
    ArgParser(const char* args)
    {
        this->parse(std::string(args));
    }


    /// @brief Prompts the user to provide input.
    /// @param prompt Optional prompt to display on the line before user input is collected.
    /// @note This clears any existing tokens the ArgParser had stored.
    void getInput(const std::string& prompt = ArgParser::empty_string)
    {
        if (prompt != ArgParser::empty_string)
        {
            std::cout << prompt << "\n";
        }
        std::cout << "> ";
        std::cin >> *this;
    }


    /// @brief Changes the parsed arguments of the ArgParser to a new string.
    /// @param args A string of arguments deliminated by a space.
    /// @note This clears any existing tokens the ArgParser had stored.
    void setArgs(const std::string& args)
    {
        this->tokens.clear();
        this->parse(args);
    }


    /// @brief Gets the command in the specified position.
    /// @param x Position to retrieve.
    /// @return A view of that position's command string if the position exists.
    ///         Empty string if the number of parsed tokens is less than X.
    const std::string& operator[](const size_t& x) const
    {
        if (this->tokens.size() <= x)
        {
            return ArgParser::empty_string;
        }
        return this->tokens[x];
    }


    /// @brief Gets the command in the specified position.
    /// @param x Position to retrieve.
    /// @return A cast of that position's command string to the requested type.
    /// @throws std::invalid_argument if the number of parsed tokens is less than X.
    template <typename T>
    T get(const size_t& x) const
    {
        const std::string& opt = this->operator[](x);
        if (opt.empty())
        {
            throw std::invalid_argument("Cannot find option  at position " + std::to_string(x) + " was not found. No default was provided.");
        }
        std::stringstream ss(opt);
        T result;
        ss >> result;
        return result;
    }


    /// @brief Gets the command in the specified position.
    /// @param x Position to retrieve.
    /// @param default_val Default value if  the number of parsed tokens is less than X.
    /// @return A cast of that position's command string to the requested type.
    ///         Or `default_val` if the number of parsed tokens is less than X.
    template <typename T>
    T get(const size_t& x, const T& default_val) const
    {
        const std::string& opt = this->operator[](x);
        if (opt.empty())
        {
            return default_val;
        }
        std::stringstream ss(opt);
        T result;
        ss >> result;
        return result;
    }


    /// @brief Checks of the option flag was passed and returns the associated value.
    /// @param option Option flag.
    /// @return A view of the command flag's value if it exists. Empty string if the flag was not included.
    const std::string& get(const std::string &option) const
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
    T get(const std::string &option) const
    {
        const std::string& opt = this->get(option);
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
    T get(const std::string &option, const T& default_val) const
    {
        const std::string& opt = this->get(option);
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
    bool has(const std::string &option) const
    {
        return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
    }


    /// @brief Clears arguments from the ArgParser and sets the token vector to one empty string.
    void clear()
    {
        this->tokens.clear();
        this->if_no_tokens_add_empty();
    }


    /// @brief True if the ArgParser has arguments to be queried. False if it is empty. This can
    ///        check if the line that was parsed was empty.
    bool hasArgs() const
    {
        return !(this->tokens.size() <= 1 && this->tokens[0] == ArgParser::empty_string);
    }


private:
    /// @brief Parses the inputs into the ArgParser's tokens
    /// @param argc Count of arguments in the argument vector.
    /// @param argv Argument array.
    void parse(const int &argc, const char **argv)
    {
        tokens.clear();
        this->tokens.reserve(argc - 1);
        for (int i=1; i < argc; ++i)
        {
            this->tokens.push_back(std::string(argv[i]));
        }
        this->if_no_tokens_add_empty();
    }


    /// @brief Parses the inputs into the ArgParser's tokens
    /// @param args A string of arguments deliminated by a space.
    void parse(const std::string& args)
    {
        static const char delim = ' ';
        tokens.clear();
        std::stringstream ss(args);
        std::string s;
        while (std::getline(ss, s, delim))
        {
            if (strings::has_contents(s))
            {
                strings::trim(s);
                this->tokens.push_back(s);
            }
        }
        this->if_no_tokens_add_empty();
    }


    /// @brief Ensures that the token vector contains one item that is an empty string in the case
    ///        that no args were parsed into it.
    void if_no_tokens_add_empty()
    {
        if (this->tokens.size() == 0)
        {
            this->tokens.push_back(ArgParser::empty_string);
        }
    }

    /// @brief Default return of an empty string.
    const static std::string empty_string;

    /// @brief Vector of each input string
    std::vector<std::string> tokens;
};


const std::string ArgParser::empty_string = std::string("");


/// @brief Reads from the input stream.
/// @param stream Input stream to read from.
/// @param parser ArgParser to read to.
/// @return Reference to the input stream.
/// @warning The parser reads a maximum 300 characters from a line in the stream.
std::istream& operator>> (std::istream& stream, ArgParser& parser)
{
    static const size_t string_buffer_size = 300;
    std::string s(string_buffer_size, ' ');
    stream.getline(&s[0], string_buffer_size);

    // Downsize the string and trim any whitespace before parsing.
    s.resize(std::char_traits<char>::length(&s[0]));
    strings::trim(s);
    parser.parse(s);

    // Flush any remaining characters from the rest of the buffer and reset any
    // flags; the failbit is set if the input steam exceeds string_buffer_size.
    if (stream.fail())
    {
        stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    stream.clear();
    return stream;
}


/// @brief Writes the contents of a ArgParser to the output stream.
/// @param out Output stream to write to.
/// @param parser ArgParser to write out.
/// @return Reference to the output stream.
std::ostream& operator<< (std::ostream &out, const ArgParser& parser)
{
    out << "[";
    auto it = parser.tokens.begin();
    while (!parser.tokens.empty())
    {
        out << *it;
        if (++it == parser.tokens.end())
        {
            break;
        }
        out << "] [";
    }
    out << "]";
    return out;
}


} // namespace forge_scan
} // namespace forge_scan

#endif // FORGE_SCAN_UTILITIES_ARG_PARSER_HPP
