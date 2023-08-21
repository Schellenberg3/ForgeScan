#ifndef FORGE_SCAN_SENSOR_IM_VIEW_HPP
#define FORGE_SCAN_SENSOR_IM_VIEW_HPP

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


/// @brief Displays the Eigen matrix like an image with OpenCV
/// @param camera Camera with an image to display.
/// @param color_map When true will apply a color map to the depth image. If false will display the
///                  depth image as greyscale. Default is false.
/// @note  This looks like OpenCV because it is. We make a copy of the Eigen matrix in
///        OpenCV and apply some scaling to view it. Distances above and below the depth
///        limits are displayed as white pixels (i.e., values set to 1).
inline void imshow_depth(const std::shared_ptr<const Camera>& camera,
                         const bool& color_map = false)
{
    static const std::string window_name = "Depth Image";

    cv::Mat cv_image;
    cv::eigen2cv(camera->image, cv_image);

    const float span = camera->intr->max_d - camera->intr->min_d;
    cv_image.forEach<cv::Vec<float, 1>>(
        [&camera, &span](cv::Vec<float, 1>& pixel, const int * position)
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

    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::imshow(window_name, cv_image);
    cv::waitKey(0);
}


} // namespace sensor
} // namespace forge_scan


#endif // FORGE_SCAN_SENSOR_IM_VIEW_HPP
