#ifndef FORGE_SCAN_UTILITIES_RANDOM_HPP
#define FORGE_SCAN_UTILITIES_RANDOM_HPP

#include <random>
#include <type_traits>

// Define M_PI in case it is not provided by the compiler.
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


namespace forge_scan {
namespace utilities {


/// @brief Pseudo-random generation of seed values for other generators.
std::random_device RANDOM_DEVICE;


template<
    typename T = float,
    typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type
>
struct RandomSampler
{
    RandomSampler(const int& seed = -1)
        : seed( seed > 0 ? static_cast<unsigned int>(seed) : RANDOM_DEVICE() ),
          gen( std::mt19937(this->seed) ),
          uniform_dist( std::uniform_real_distribution<double>(0.0, 1.0) )
    {

    }

    /// @brief Returns uniformly sampled value between [0, 1)
    T uniform()
    {
        return static_cast<T>(uniform_dist(gen));
    }

    /// @brief Returns a uniformly sampled value between [0, scale)
    /// @param scale Value to scale the uniform distribution by.
    T uniform(const T& scale)
    {
        return static_cast<T>(uniform_dist(gen) * scale);
    }


    /// @brief Returns a uniformly sampled value between the two values.
    ///        This will correct
    /// @param x1 The first end of the range.
    /// @param x2 The second end of the range.
    T uniform(const T& x1, const T& x2)
    {
        if (x2 < x1)
        {
            return static_cast<T>( (uniform_dist(gen) * (x1 - x2)) + x2 );
        }
        else if (x2 > x1)
        {
            return static_cast<T>( (uniform_dist(gen) * (x2 - x1)) + x1 );
        }
        return x1;
    }


    /// @brief Uniformly samples angle values from a unit sphere. In Radians.
    /// @param theta Output: angle around the positive X-axis. [0, 2*PI)
    /// @param phi   Output: angle from positive Z-axis. [0, PI)
    /// @param phi_negative If true will sample phi in (-PI, PI). Default false.
    void sphere(T& theta, T& phi, const bool& phi_negative = false)
    {
        theta = static_cast<T>(2 * M_PI * uniform_dist(gen));          ///  0  - 2*PI  radians in theta
        phi   = static_cast<T>(std::acos(1 - 2 * uniform_dist(gen)));  ///  0  -  PI   radians in phi
        if (phi_negative && uniform_dist(gen) < 0.5)
        {
            phi *= -1;                                                 /// -PI -  PI   radians in phi
        }
    }

    /// @brief Uniformly samples angle values from a unit sphere.
    /// @param theta Output: angle around the positive X-axis. [0, 360)
    /// @param phi   Output: angle from positive Z-axis. [0, 180)
    /// @param phi_negative If true will sample phi in (-180, 180). Default false.
    void sphereDegrees(T& theta, T& phi, const bool& phi_negative = false) {
        theta = static_cast<T>(2 * M_PI * uniform_dist(gen));          ///   0  - 360 degrees in theta
        phi   = static_cast<T>(std::acos(1 - 2 * uniform_dist(gen)));  ///   0  - 180 degrees in phi
        if (phi_negative && uniform_dist(gen) < 0.5)
        {
            phi *= -1;                                                 /// -180 - 180 degrees in phi
        }
    }


    /// @brief Seed used to create this generator.
    const unsigned int seed;

    /// @brief Random number engine for performing sampling on the uniform real distribution.
    std::mt19937 gen;

private:
    /// @brief Uniform distribution of double values over [0, 1).
    std::uniform_real_distribution<double> uniform_dist;
};


} // namespace utilities
} // namespace forge_scan


#endif // FORGE_SCAN_UTILITIES_RANDOM_HPP
