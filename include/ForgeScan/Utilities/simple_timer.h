#ifndef FORGESCAN_UTILITIES_SIMPLE_TIMER_H
#define FORGESCAN_UTILITIES_SIMPLE_TIMER_H

#include <chrono>

#define MILISECONDS_TO_SECONDS 0.001


namespace ForgeScan {
namespace Utilities {


/// @brief Minimal implementation for a timer using std::chrono.
class SimpleTimer {
public:
    /// @brief Starts the SimpleTimer.
    void start() {
        start_time = std::chrono::system_clock::now();
        running = true;
    }

    /// @brief Ends the SimpleTimer.
    void stop() {
        end_time = std::chrono::system_clock::now();
        running = false;
    }

    /// @brief  Calculates the elapsed time in miliseconds, even if the timer is running. 
    /// @return Elapsed time in miliseconds.
    long long elapsedMilliseconds() {
        if (running) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count();
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    }

    /// @brief  Calculates the elapsed time in seconds, even if the timer is running. 
    /// @return Elapsed time in seconds.
    double elapsedSeconds() { return elapsedMilliseconds() * MILISECONDS_TO_SECONDS; }

private:
    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::time_point<std::chrono::system_clock> end_time;
    bool running = false;
};


} // Utilities
} // ForgeScan

#endif // FORGESCAN_UTILITIES_SIMPLE_TIMER_H
