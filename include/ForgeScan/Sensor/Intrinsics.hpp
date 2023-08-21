#ifndef FORGE_SCAN_SENSOR_INTRINSICS_HPP
#define FORGE_SCAN_SENSOR_INTRINSICS_HPP

#include <memory>

#include "ForgeScan/Common/Types.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"


namespace forge_scan {
namespace sensor {


/// @brief Intrinsic properties of a depth Camera.
struct Intrinsics
{
    // ***************************************************************************************** //
    // *                                 STATIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Creates a shared pointer to an Intrinsics struct.
    /// @note Default values mimic those of an Intel RealSense D455 camera.
    /// @return Shared pointer to an Intrinsics struct.
    static std::shared_ptr<Intrinsics> create()
    {
        return std::shared_ptr<Intrinsics>(new Intrinsics(1280, 720, 0.6, 6, 87, 58));
    }


    /// @brief Creates a shared pointer to an Intrinsics struct.
    /// @param width   Sensor dimension, number of of X-pixels.
    /// @param height  Sensor dimension, number of of Y-pixels.
    /// @param min_d   Minimum depth.
    /// @param max_d   Maximum depth.
    /// @param fov_x_deg Field of view in X.
    /// @param fov_y_deg Field of view in Y.
    /// @return Shared pointer to an Intrinsics struct.
    static std::shared_ptr<Intrinsics> create(const size_t& width, const size_t& height,
                                              const float& min_d, const float& max_d,
                                              const float& fov_x_deg, const float& fov_y_deg)
    {
        return std::shared_ptr<Intrinsics>(new Intrinsics(width, height, min_d, max_d, fov_x_deg, fov_y_deg));
    }


    /// @brief Creates a shared pointer to an Intrinsics struct.
    /// @param width   Sensor dimension, number of of X-pixels.
    /// @param height  Sensor dimension, number of of Y-pixels.
    /// @param min_d   Minimum depth.
    /// @param max_d   Maximum depth.
    /// @param fov_deg Field of view in the diagonal.
    /// @return Shared pointer to an Intrinsics struct.
    static std::shared_ptr<Intrinsics> create(const size_t& width, const size_t& height,
                                              const float& min_d, const float& max_d,
                                              const float& fov_deg)
    {
        return std::shared_ptr<Intrinsics>(new Intrinsics(width, height, min_d, max_d, fov_deg));
    }


    /// @brief Creates a shared pointer to an Intrinsics struct.
    /// @param width  Sensor dimension, number of of X-pixels.
    /// @param height Sensor dimension, number of of Y-pixels.
    /// @param min_d  Minimum depth.
    /// @param max_d  Maximum depth.
    /// @param K      Intrinsic matrix for the camera.
    /// @return Shared pointer to an Intrinsics struct.
    static std::shared_ptr<Intrinsics> create(const size_t& width, const size_t& height,
                                              const float& min_d, const float& max_d,
                                              const Eigen::Matrix3f& K)
    {
        return std::shared_ptr<Intrinsics>(new Intrinsics(width, height, min_d, max_d, K));
    }


    /// @brief Creates a shared pointer to an Intrinsics struct.
    /// @param parser Arg Parser with arguments to construct Grid Properties from.
    /// @return Shared pointer to an Intrinsics struct.
    static std::shared_ptr<Intrinsics> create(const utilities::ArgParser& parser)
    {
        if (parser.cmdOptionExists("--fov"))
        {
            return std::shared_ptr<Intrinsics>(new Intrinsics(parser.getCmdOption<float>("--width", 1280),
                                                              parser.getCmdOption<float>("--height", 720),
                                                              parser.getCmdOption<float>("--min-d", 0.0),
                                                              parser.getCmdOption<float>("--max-d", 10),
                                                              parser.getCmdOption<float>("--fov",   80)));
        }
        else if (parser.cmdOptionExists("--d455"))
        {
            float scale =  std::min( std::max(parser.getCmdOption<float>("--d455", 1.0f), 0.01f), 2.0f);
            return std::shared_ptr<Intrinsics>(new Intrinsics(1280 * scale, 780 * scale, 0.6, 6.0, 87, 58));
        }
        else
        {
            return std::shared_ptr<Intrinsics>(new Intrinsics(parser.getCmdOption<float>("--width", 1280),
                                                              parser.getCmdOption<float>("--height", 720),
                                                              parser.getCmdOption<float>("--min-d", 0.0),
                                                              parser.getCmdOption<float>("--max-d", 10),
                                                              parser.getCmdOption<float>("--fov-x", 87),
                                                              parser.getCmdOption<float>("--fov-y", 58)));
        }
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Returns the intrinsic matrix.
    /// @return Eigen matrix of the Camera Intrinsic properties.
    Eigen::Matrix3f getMatrix() const
    {
        Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
        K(0, 0) = f_x;
        K(1, 1) = f_y;
        K(0, 2) = c_x;
        K(1, 2) = c_y;
        return K;
    }


    /// @brief Sets focal lengths and center offsets based on the provided intrinsic matrix.
    /// @param K New intrinsic matrix. Assumed to be valid.
    void setFromMatrix(const Eigen::Matrix3f& K)
    {
        f_x = K(0, 0);
        f_y = K(1, 1);
        c_x = K(0, 2);
        c_y = K(1, 2);
    }


    /// @brief Calculates how many pixels the Depth Camera has.
    /// @return Total number of pixels in the camera. 
    size_t size() const
    {
        return this->width * this->height;
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Sensor shape: width is the number of X-pixels and height is the number if Y-pixels.
    size_t width, height;

    /// @brief Maximum and minimum depth for the Camera.
    float min_d, max_d;

    /// @brief Focal length for the Camera.
    float f_x = 0, f_y = 0;

    /// @brief Principle point offset for the image.
    float c_x = 0, c_y = 0;


private:
    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce use of shared pointers.
    /// @details The default value approximate those of the Intel RealSense d455 camera.
    /// @param width   Sensor dimension, number of of X-pixels.
    /// @param height  Sensor dimension, number of of Y-pixels.
    /// @param min_d   Minimum depth.
    /// @param max_d   Maximum depth.
    /// @param fov_x_deg Field of view in X.
    /// @param fov_x_deg Field of view in Y.
    Intrinsics(const size_t& width, const size_t& height,
               const float& min_d, const float& max_d,
               const float& fov_x_deg, const float& fov_y_deg)
        : width(width),
          height(height),
          min_d(min_d),
          max_d(max_d),
          f_x(this->getFocalLength(this->width,  fov_x_deg)),
          f_y(this->getFocalLength(this->height, fov_y_deg)),
          c_x(0.5 * this->width),
          c_y(0.5 * this->height)
    {
        this->checkDepth();
    }


    /// @brief Private constructor to enforce use of shared pointers.
    /// @details The default value approximate those of the Intel RealSense d455 camera.
    /// @param width   Sensor dimension, number of of X-pixels.
    /// @param height  Sensor dimension, number of of Y-pixels.
    /// @param min_d   Minimum depth.
    /// @param max_d   Maximum depth.
    /// @param fov_deg Field of view in Diagonal.
    Intrinsics(const size_t& width, const size_t& height, const float& min_d, const float& max_d, const float& fov_deg)
        : width(width),
          height(height),
          min_d(min_d),
          max_d(max_d),
          f_x(this->getFocalLength(this->width, fov_deg)),
          f_y(this->getFocalLength(this->width, fov_deg)),
          c_x(0.5 * this->width),
          c_y(0.5 * this->height)
    {
        this->checkDepth();
    }

    /// @brief Private constructor to enforce use of shared pointers.
    /// @param width  Sensor dimension, number of of X-pixels.
    /// @param height Sensor dimension, number of of Y-pixels.
    /// @param min_d  Minimum depth.
    /// @param max_d  Maximum depth.
    /// @param K      Intrinsic matrix for the camera.
    Intrinsics(const size_t& width, const size_t& height, const float& min_d, const float& max_d, const Eigen::Matrix3f& K)
        : width(width),
          height(height),
          min_d(min_d),
          max_d(max_d)
    {
        this->setFromMatrix(K);
        this->checkDepth();
    }


    /// @brief Ensures min_d is less than max_d and sets them to default values if they were below zero. (0 and 10, respectively).
    void checkDepth()
    {
        if (this->min_d <= 0) this->min_d = 0;

        if (this->max_d <= 0) this->max_d = 10;

        if (this->min_d > this->max_d) std::swap(this->min_d, this->max_d);
    }



    // ***************************************************************************************** //
    // *                                PRIVATE STATIC METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Calculates the focal length in the units of the distance.
    ///        See: https://en.wikipedia.org/wiki/Angle_of_view#Calculating_a_camera's_angle_of_view
    /// @param distance Distance (e.g., horizontal, vertical or diagonal) in the same units as focal length (world units or pixels).
    /// @param fov_deg  Field of view in the same direction as the distance.
    static float getFocalLength(const float& distance, const float& fov_deg)
    {
        return 0.5 * distance  / std::tan(0.5 * (M_PI / 180) * fov_deg);
    }
};


} // namespace sensor
} // namespace forge_scan


#endif // FORGE_SCAN_SENSOR_INTRINSICS_HPP
