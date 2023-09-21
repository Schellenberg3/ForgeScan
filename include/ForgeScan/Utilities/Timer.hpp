#ifndef FORGE_SCAN_UTILITIES_TIMER_HPP
#define FORGE_SCAN_UTILITIES_TIMER_HPP

#include <chrono>

// Conversion of miliseconds to seconds. Defined for this header file only.
#define MILISECONDS_TO_SECONDS 0.001


namespace forge_scan {
namespace utilities {


/// @brief Minimal implementation for a timer using std::chrono.
struct Timer
{
    /// @brief Starts the SimpleTimer.
    void start()
    {
        start_time = std::chrono::system_clock::now();
        running = true;
    }

    /// @brief Stops the SimpleTimer.
    void stop()
    {
        end_time = std::chrono::system_clock::now();
        running = false;
    }

    /// @brief  Calculates the elapsed time in miliseconds, even if the timer is running.
    /// @return Elapsed time in miliseconds.
    long long elapsedMilliseconds()
    {
        if (this->running)
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count();
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    }


    /// @brief  Calculates the elapsed time in microseconds, even if the timer is running.
    /// @return Elapsed time in microseconds.
    long long elapsedMicroseconds()
    {
        if (this->running)
        {
            return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start_time).count();
        }
        return std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    }


    /// @brief  Calculates the elapsed time in seconds, even if the timer is running.
    /// @return Elapsed time in seconds.
    double elapsedSeconds()
    {
        return this->elapsedMilliseconds() * MILISECONDS_TO_SECONDS;
    }


private:

    /// @brief Start time for the timer.
    std::chrono::time_point<std::chrono::system_clock> start_time;

    /// @brief End time for the timer.
    std::chrono::time_point<std::chrono::system_clock> end_time;

    /// @brief True if the timer is running.
    bool running = false;
};


} // namespace utilities
} // namespace forge_scan


#ifdef MILISECONDS_TO_SECONDS
    #undef MILISECONDS_TO_SECONDS
#endif


#endif // FORGE_SCAN_UTILITIES_TIMER_HPP
