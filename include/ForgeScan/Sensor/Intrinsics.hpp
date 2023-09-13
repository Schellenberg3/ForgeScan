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
    /// @param parser ArgParser with arguments to construct Intrinsics from.
    /// @return Shared pointer to an Intrinsics struct.
    static std::shared_ptr<Intrinsics> create(const utilities::ArgParser& parser)
    {
        if (parser.has(Intrinsics::parse_d455))
        {
            float scale =  std::clamp(parser.get<float>(Intrinsics::parse_d455, 1.0f), 0.01f, 2.0f);
            return std::shared_ptr<Intrinsics>(new Intrinsics(Intrinsics::default_width  * scale,
                                                              Intrinsics::default_height * scale,
                                                              Intrinsics::realsense_min_d,
                                                              Intrinsics::realsense_max_d,
                                                              Intrinsics::default_fov_x,
                                                              Intrinsics::default_fov_y));
        }
        else if (parser.has(Intrinsics::parse_fov))
        {
            return std::shared_ptr<Intrinsics>(new Intrinsics(parser.get<size_t>(Intrinsics::parse_width,  Intrinsics::default_width),
                                                              parser.get<size_t>(Intrinsics::parse_height, Intrinsics::default_height),
                                                              parser.get<float>(Intrinsics::parse_min_d,   Intrinsics::default_min_d),
                                                              parser.get<float>(Intrinsics::parse_max_d,   Intrinsics::default_max_d),
                                                              parser.get<float>(Intrinsics::parse_fov,     Intrinsics::default_fov)));
        }
        else
        {
            return std::shared_ptr<Intrinsics>(new Intrinsics(parser.get<size_t>(Intrinsics::parse_width,  Intrinsics::default_width),
                                                              parser.get<size_t>(Intrinsics::parse_height, Intrinsics::default_height),
                                                              parser.get<float>(Intrinsics::parse_min_d,   Intrinsics::default_min_d),
                                                              parser.get<float>(Intrinsics::parse_max_d,   Intrinsics::default_max_d),
                                                              parser.get<float>(Intrinsics::parse_fov_x,   Intrinsics::default_fov_x),
                                                              parser.get<float>(Intrinsics::parse_fov_y,   Intrinsics::default_fov_y)));
        }
    }


    /// @return Help message for creating Intrinsics with ArgParser.
    static std::string helpMessage()
    {
        return "Intrinsics may be may be in one of three ways:"
               "\n\t(1) " + Intrinsics::help_string_1 +
               "\n\t(2) " + Intrinsics::help_string_2 +
               "\n\t(3) " + Intrinsics::help_string_3 +
               "\nIf the optional arguments are not provided, the default values are:"
               "\n\t(1) " + Intrinsics::default_arguments_1+
               "\n\t(2) " + Intrinsics::default_arguments_2 +
               "\n\t(3) " + Intrinsics::default_arguments_3;
    }

    static const size_t default_width, default_height;

    static const float default_min_d, default_max_d, default_fov, default_fov_x, default_fov_y,
                       realsense_min_d, realsense_max_d;

    static const std::string parse_width, parse_height, parse_min_d, parse_max_d,
                             parse_fov, parse_fov_x, parse_fov_y, parse_d455;

    static const std::string help_string_1, help_string_2, help_string_3,
                             default_arguments_1, default_arguments_2, default_arguments_3;


    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Returns the Intrinsic matrix.
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


    /// @brief Sets focal lengths and center offsets based on the provided Intrinsic matrix.
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


/// @brief Writes the contents of a Intrinsics to the output stream.
/// @param out Output stream to write to.
/// @param intr Intrinsics to write out.
/// @return Reference to the output stream.
std::ostream& operator<< (std::ostream &out, const Intrinsics& intr)
{
    out << "(" << intr.width << ", "<< intr.height << ") pixels "
        << "min_d=" << intr.min_d << " max_d=" << intr.max_d << " "
        << "fx=" << intr.f_x << " fy=" << intr.f_x << " "
        << "cx=" << intr.c_x << " cy=" << intr.c_x;
    return out;
}


/// @brief Default width and height for pixels on a sensor. Uses RealSense d455 properties.
const size_t Intrinsics::default_width = 1280,
             Intrinsics::default_height = 720;

/// @brief Default minimum and maximum depth for a sensor.
const float Intrinsics::default_min_d = 0.0f,
            Intrinsics::default_max_d = 10.0f;

/// @brief Default minimum and maximum depth for a RealSense d455.
const float Intrinsics::realsense_min_d = 0.6f,
            Intrinsics::realsense_max_d = 6.0f;

/// @brief Default field of view properties for a sensor. Uses RealSense d455 properties.
const float Intrinsics::default_fov = 80.0f,
            Intrinsics::default_fov_x = 87.0f,
            Intrinsics::default_fov_y = 58.0f;

/// @brief ArgParser key for number of pixels in the X (width) and Y (height) directions.
const std::string Intrinsics::parse_width = "--width",
                  Intrinsics::parse_height = "--height";

/// @brief ArgParser key for minimum and maximum depth.
const std::string Intrinsics::parse_min_d = "--min-d",
                  Intrinsics::parse_max_d = "--max-d";

/// @brief ArgParser key for field of view properties.
const std::string Intrinsics::parse_fov = "--fov",
                  Intrinsics::parse_fov_x = "--fov-x",
                  Intrinsics::parse_fov_y = "--fov-y";

/// @brief ArgParser flag to use Intel RealSense d455 properties and ArgParser key for how
///        much to scale the number of pixels used.
const std::string Intrinsics::parse_d455 = "--d455";

/// @brief String explaining what arguments this class accepts for the RealSense option.
const std::string Intrinsics::help_string_1 =
    Intrinsics::parse_d455 + " [pixel resolution scale value]";

/// @brief String explaining what this class's default parsed values for the RealSense option.
const std::string Intrinsics::default_arguments_1 =
    Intrinsics::parse_d455 + " " + std::to_string(1.0f);


/// @brief String explaining what arguments this class accepts for the single FOV option.
const std::string Intrinsics::help_string_2 =
    "["  + Intrinsics::parse_width + " <pixels in x>]"
    " [" + Intrinsics::parse_height + " <pixels in y>]"
    " [" + Intrinsics::parse_min_d + " <depth min>]"
    " [" + Intrinsics::parse_max_d + " <depth max>]"
    " [" + Intrinsics::parse_fov + " <horizontal FOV degrees>]";

/// @brief String explaining what this class's default parsed values for the single FOV option.
const std::string Intrinsics::default_arguments_2 =
    Intrinsics::parse_width + " " + std::to_string(Intrinsics::default_width) +
    Intrinsics::parse_height + " " + std::to_string(Intrinsics::default_height) +
    Intrinsics::parse_min_d + " " + std::to_string(Intrinsics::default_min_d) +
    Intrinsics::parse_max_d + " " + std::to_string(Intrinsics::default_max_d) +
    Intrinsics::parse_fov + " " + std::to_string(Intrinsics::default_fov);

/// @brief String explaining what arguments this class accepts for the independent FOV option.
const std::string Intrinsics::help_string_3 =
    "["  + Intrinsics::parse_width + " <pixels in x>]" +
    " [" + Intrinsics::parse_height + " <pixels in y>]" +
    " [" + Intrinsics::parse_min_d + " <depth min>]" +
    " [" + Intrinsics::parse_max_d + " <depth max>]" +
    " [" + Intrinsics::parse_fov_x + " <X FOV degrees>]" +
    " [" + Intrinsics::parse_fov_y + " <Y FOV degrees>]";

/// @brief String explaining what this class's default parsed values for the independent FOV option.
const std::string Intrinsics::default_arguments_3 =
    Intrinsics::parse_width + " " + std::to_string(Intrinsics::default_width) +
    Intrinsics::parse_height + " " + std::to_string(Intrinsics::default_height) +
    Intrinsics::parse_min_d + " " + std::to_string(Intrinsics::default_min_d) +
    Intrinsics::parse_max_d + " " + std::to_string(Intrinsics::default_max_d) +
    Intrinsics::parse_fov_x + " " + std::to_string(Intrinsics::default_fov_x) +
    Intrinsics::parse_fov_y + " " + std::to_string(Intrinsics::default_fov_y);


} // namespace sensor
} // namespace forge_scan


#endif // FORGE_SCAN_SENSOR_INTRINSICS_HPP
