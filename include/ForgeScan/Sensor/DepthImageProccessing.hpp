#ifndef FORGE_SCAN_SENSOR_DEPTH_IMAGE_PROCESSING_HPP
#define FORGE_SCAN_SENSOR_DEPTH_IMAGE_PROCESSING_HPP

#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "ForgeScan/Sensor/Camera.hpp"


namespace forge_scan {
namespace sensor {


/// @brief Provides an interface between the Eigen matrix a Camera object uses to store depth and
///        the OpenCV library.
/// @note  While technically a `struct`, this only has static functions so it acts more like a
///        namespace with priviledged access to Camera's internal information.
struct DepthImageProcessing
{
    /// @brief Converts a Camera's depth image to an OpenCV image. Applies a color map if requested.
    /// @param camera    Camera with an image to display.
    /// @param color_map When true will apply a color map to the depth image. If false will display
    ///                  the depth image as greyscale. Default is false.
    /// @returns OpenCV image.
    static inline cv::Mat depth_image_to_opencv(const std::shared_ptr<const Camera>& camera,
                                                const bool& color_map = false)
    {
        static const std::string window_name = "Depth Image";

        cv::Mat cv_image;
        cv::eigen2cv(camera->image, cv_image);

        const float span = camera->intr->max_d - camera->intr->min_d;
        cv_image.forEach<cv::Vec<float, 1>>(
            [&camera, &span](cv::Vec<float, 1>& pixel, const int * /*position*/)
            {
                pixel[0] -= camera->intr->min_d;
                pixel[0] /= span;
                pixel[0]  = pixel[0] < 0 ? 1 : pixel[0];
            }
        );

        if (color_map)
        {
            cv_image.convertTo(cv_image, CV_8UC1 , 255);
            cv::applyColorMap(cv_image, cv_image, cv::COLORMAP_TURBO);
        }

        return cv_image;
    }


    /// @brief Displays the Camera's depth image with OpenCV, pausing until the user hits a key.
    /// @param camera      Camera with an image to display.
    /// @param color_map   When true will apply a color map to the depth image. If false will display
    ///                    the depth image as greyscale. Default is false.
    /// @param window_name Name and identifier for the window.
    static inline void imshow(const std::shared_ptr<const Camera>& camera,
                              const bool& color_map = false,
                              const std::string& window_name = "Depth Image")
    {
        cv::Mat cv_image = depth_image_to_opencv(camera, color_map);

        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name, cv_image);
        cv::waitKey(0);
    }


    /// @brief Saves the Camera's depth image with OpenCV.
    /// @param camera Camera with an image to display.
    /// @param fname  Filename for the image.
    static inline void imwrite(const std::shared_ptr<const Camera>& camera,
                               const std::filesystem::path& fname)
    {
        cv::Mat cv_image = depth_image_to_opencv(camera, true);
        cv::imwrite(fname.string(), cv_image);
    }
};

} // namespace sensor
} // namespace forge_scan


#endif // FORGE_SCAN_SENSOR_DEPTH_IMAGE_PROCESSING_HPP
