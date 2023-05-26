#include <string>
#include <vector>
#include <algorithm>

/// @brief   A simple argument parsing tool.
/// @details Adapted from:
///              https://stackoverflow.com/questions/865668/#868894
class ArgParser{
public:
    /// @brief Constructs the argument parser from the argument count and argument array.
    /// @param argc Count of arguments in the argument vector
    /// @param argv Argument array
    ArgParser (int &argc, char **argv){
        for (int i=1; i < argc; ++i)
            this->tokens.push_back(std::string(argv[i]));
    }

    /// @brief Checks of the option flag was passed and returns the associated value.
    /// @param option Option flag.
    /// @return Flag value if it exists. Empty string if the flag was not included.
    const std::string& getCmdOption(const std::string &option) const{
        std::vector<std::string>::const_iterator itr;
        itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end()){
            return *itr;
        }
        static const std::string empty_string("");
        return empty_string;
    }

    /// @brief Checks if the specific option was included in the command line input.
    /// @param option Option to search check for.
    /// @return True if the option exists. False else.
    bool cmdOptionExists(const std::string &option) const{
        return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
    }

private:
    /// @brief Vector of each input string
    std::vector <std::string> tokens;
};