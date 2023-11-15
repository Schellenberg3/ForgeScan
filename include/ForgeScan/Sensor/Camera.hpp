#ifndef FORGE_SCAN_SENSOR_DEPTH_CAMERA_HPP
#define FORGE_SCAN_SENSOR_DEPTH_CAMERA_HPP

#include <cmath>
#include <memory>
#include <stdexcept>

#include "ForgeScan/Common/Entity.hpp"
#include "ForgeScan/Sensor/Intrinsics.hpp"

#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Random.hpp"


namespace forge_scan {
namespace simulation {

    // Forward definition to allow friend access.
    struct Scene;

} // namespace simulation
} // namespace forge_scan


namespace forge_scan {
namespace sensor     {


/// @brief Simulates a simple pinhole depth camera. Stores a depth image in an Eigen Matrix.
///        May be used with the Scene class to generate synthetic data.
struct Camera : public Entity
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @details Required to access the depth image and Intrinsics when converting an Eigen matrix
    ///        to an OpenCV image.
    friend struct DepthImageProcessing;

    /// @details Required to directly access the depth image when generating a synthetic image.
    friend struct simulation::Scene;


    // ***************************************************************************************** //
    // *                                 STATIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Creates a shared pointer to a Camera.
    /// @param intr Intrinsic properties for the Camera.
    /// @param extr Extrinsic pose of the Camera.
    /// @param percent_noise Amount of gaussian noise to add.
    /// @param seed Seed for the noise RNG. Default -1 will use a random seed.
    /// @return Shared pointer to a Camera.
    static std::shared_ptr<Camera> create(const std::shared_ptr<const Intrinsics>& intr = Intrinsics::create(),
                                          const float& percent_noise = 0.02,
                                          const float& seed = -1,
                                          const Extrinsic& extr = Extrinsic::Identity())
    {
        return std::shared_ptr<Camera>(new Camera(intr, percent_noise, seed, extr));
    }


    /// @brief Static method to extract a 3D point from a pixel in a depth image.
    /// @param intr  Intrinsics for the Camera.
    /// @param image Depth image.
    /// @param row Pixel row in the image.
    /// @param col Pixel column in the image.
    /// @param [out] out Output point.
    /// @note The ouput point is implicitly relative to the optical frame of the camera
    ///       that generated the depth image.
    /// @throws std::invalid_argument If the pixel was out of bounds.
    static void getPoint(const std::shared_ptr<Intrinsics>& intr,
                         const DepthImage& image, const Eigen::Index& row, const Eigen::Index col,
                         Point& out)
    {
        throwIfPixelIsInvalid(image, row, col);
        out.z() = image(row, col);
        out.y() = (row - intr->c_y) * out.z() / intr->f_y;
        out.x() = (col - intr->c_x) * out.z() / intr->f_x;
    }


    /// @brief Static method for turning a depth image and Intrinsic properties into a list of Points.
    /// @param intr  Intrinsics for the Camera that took the image.
    /// @param image Depth image.
    /// @param [out] dest Matrix to store the depth image's Points in.
    /// @note The ouput points are implicitly relative to the optical frame of the camera
    ///       that generated the depth image.
    static void getPointsFromImageAndIntrinsics(const std::shared_ptr<Intrinsics>& intr,
                                                const DepthImage& image,
                                                PointMatrix& dest)
    {
        dest.setConstant(3, intr->size(), 0);
        size_t n = 0;
        Point xyz;
        for (Eigen::Index row = 0; row < dest.rows(); ++row)
        {
            for (Eigen::Index col = 0; col < dest.cols(); ++col)
            {
                getPoint(intr, image, row, col, xyz);
                dest.col(n) << xyz;
                ++n;
            }
        }
    }


    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //



    /// @brief Orients the sensor to point at the specified point.
    /// @param target Point to align the Camera's principle axis to, relative to the camera's frame.
    void orientPrincipleAxis(const Point target)
    {
        const static Ray principle_axis = Eigen::Vector3f::UnitZ();
        Extrinsic Tws(Eigen::Quaternionf().setFromTwoVectors(principle_axis, target - this->extr.translation()));
        this->transformBodyFrame(Tws);
    }


    /// @brief Turns the Camera's depth image into a list of Points, relative to the camera's frame.
    /// @param [out] dest Matrix to store the depth image's Points in.
    void getPointMatrix(PointMatrix& dest)
    {
        dest.setConstant(3, this->intr->size(), 0);
        size_t n = 0;
        Point xyz;
        for (size_t row = 0; row < this->intr->height; ++row)
        {
            for (size_t col = 0; col < this->intr->width; ++col)
            {
                this->getPoint(row, col, xyz);
                dest.col(n) << xyz;
                ++n;
            }
        }
    }


    /// @brief Extracts a 3D point from a pixel in a depth image.
    /// @param row Pixel row in the image.
    /// @param col Pixel column in the image.
    /// @returns The Point represented by the specified pixel, relative to the camera's frame.
    /// @throws std::invalid_argument If the pixel was out of bounds.
    Point getPoint(const size_t& row, const size_t col)
    {
        Point out;
        getPoint(row, col, out);
        return out;
    }


    /// @brief Extracts a 3D point from the depth image.
    /// @param row Row of the image.
    /// @param col Column of the image.
    /// @param [out] out The Point represented by the specified pixel, relative to the camera's frame.
    /// @throws std::invalid_argument If the pixel was out of bounds.
    void getPoint(const size_t& row, const size_t col, Point& out)
    {
        this->throwIfPixelIsInvalid(row, col);
        out.z() = this->image(row, col);
        out.y() = (row - this->intr->c_y) * out.z() / this->intr->f_y;
        out.x() = (col - this->intr->c_x) * out.z() / this->intr->f_x;
    }


    /// @brief Sets all pixels in the depth image to the provided value.
    /// @param value What depth value each pixel should hold.
    void resetDepth(const float& value)
    {
        this->image.setConstant(this->intr->height, this->intr->width, value);
    }


    /// @brief Resets the depth image the maximum depth value.
    void resetDepthMax()
    {
        this->resetDepth(this->intr->max_d);
    }


    /// @brief Resets the depth image the minimum depth value.
    void resetDepthMin()
    {
        this->resetDepth(this->intr->min_d);
    }


    /// @brief Adds noise to the image. The noise is gaussian with zero mean and scales the ray
    ///        by some percent of its original length.
    /// @param percent Uniform sampling factor. The ray is scaled a random percentage between [-p, +p]
    /// @note This does ensure no value is set below zero.
    /// @warning A percent value above `0.2` may cause ray tracing failures. I have not investigated why yet.
    ///          Likely some numeric instability leading to negative voxel coordinates.
    void addNoise(const float& percent)
    {
        std::uniform_real_distribution<float> ud(-1 * percent, percent);
        for (size_t row = 0; row < this->intr->height; ++row)
        {
            for (size_t col = 0; col < this->intr->width; ++col)
            {
                auto& x = this->image(row, col);
                x = std::clamp(x, this->intr->min_d, this->intr->max_d);
                x = std::max(0.0f, x + x * ud(this->sample.gen));
            }
        }
    }


    /// @brief Adds noise to the image. The noise is gaussian with zero mean and scales the ray
    ///        by some percent of its original length.
    void addNoise()
    {
        this->addNoise(this->percent_noise);
    }


    /// @brief Any depth values below the intrinsic's minimum depth are set to 0 instead.
    void saturateDepth()
    {
        for (size_t row = 0; row < this->intr->height; ++row)
        {
            for (size_t col = 0; col < this->intr->width; ++col)
            {
                auto& x = this->image(row, col);
                x = std::clamp(x, this->intr->min_d, this->intr->max_d);
            }
        }
    }


    /// @brief Sets new Intrinsic parameters for the camera.
    ///        The stored image is automatically resized and cleared.
    void setIntr(const std::shared_ptr<Intrinsics>& new_intr)
    {
        this->intr = new_intr;
        this->resetDepthMax();
    }


    /// @brief Read-only access to the Camera's current intrinsics.
    const std::shared_ptr<const Intrinsics>& getIntr() const
    {
        return this->intr;
    }


    /// @brief Read-only access to the Camera's current image.
    const DepthImage& getImage() const
    {
        return this->image;
    }


    /// @brief The depth image which the camera takes
    DepthImage image;


private:
    /// @brief Private constructor to enforce use of shared pointers.
    /// @param intr Intrinsic properties for the Camera.
    /// @param extr Extrinsic pose of the Camera.
    
    explicit Camera(const std::shared_ptr<const Intrinsics>& intr, 
                    const float& percent_noise = 0.02,
                    const float& seed = -1,
                    const Extrinsic& extr = Extrinsic::Identity())
        : Entity(extr),
          intr(intr),
          percent_noise(percent_noise),
          sample(seed)
    {
        this->resetDepthMax();
    }


    /// @brief Throws an error if the requested pixel is beyond the bounds of the image.
    /// @param intr Intrinsic properties for the camera.
    /// @param row Row location to check.
    /// @param col Column location to check.
    /// @throws std::invalid_argument If the pixel was out of bounds.
    void throwIfPixelIsInvalid(const size_t& row, const size_t col)
    {
        if (row >= this->intr->height || col >= this->intr->width)
        {
            throw std::invalid_argument("Requested pixel is beyond the bounds of the image.");
        }
    }


    /// @brief Throws an error if the requested pixel is beyond the bounds of the image.
    /// @param image Depth image of a specific size.
    /// @param row Row location to check.
    /// @param col Column location to check.
    /// @throws std::invalid_argument If the pixel was out of bounds.
    static void throwIfPixelIsInvalid(const DepthImage& image,
                                      const Eigen::Index& row, const Eigen::Index col)
    {
        if (row >= image.rows() || col >= image.cols())
        {
            throw std::invalid_argument("Requested pixel is beyond the bounds of the image.");
        }
    }


    /// @brief Pointer to the derived class's intrinsic parameters.
    std::shared_ptr<const Intrinsics> intr;

    /// @brief Amount of noise to add.
    float percent_noise;

    /// @brief Amount of noise to add.
    utilities::RandomSampler<float> sample;
};


} // namespace sensor
} // namespace forge_scan


#endif // FORGE_SCAN_SENSOR_DEPTH_CAMERA_HPP
