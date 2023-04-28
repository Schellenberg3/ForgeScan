#ifndef FORGESCAN_SENSOR_READING_H
#define FORGESCAN_SENSOR_READING_H

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/sensor_intrinsics.h>

#include <Eigen/Geometry>

#include <random>
#include <stdexcept>


static constexpr float FLOATING_POINT_INF = std::numeric_limits<float>::infinity();


/// @brief Data element for depth sensors which stores position in spherical coordinates.
struct DepthSensorPoint
{
    /// @brief Depth value for the point
    float depth;

    /// @brief Angle in radians from the positive Z-axis around the positive X-axis (theta) and positive Y-axis (phi). 
    float theta, phi;

    /// @brief Default constructor. Places the depth at infinity and zero for theta and phi.
    DepthSensorPoint() : depth(FLOATING_POINT_INF), theta(0), phi(0) {}

    /// @brief Creates a sensed point at the provided spherical position.
    /// @param depth Depth at the point.
    /// @param theta Angle in radians from the positive Z-axis around the positive X-axis.
    /// @param phi Angle in radians from the positive Z-axis around the positive Y-axis.
    DepthSensorPoint(const float& depth, const float& theta, const float& phi) : 
        depth(depth), theta(theta), phi(phi)
        { }
    
    /// @brief Calculates the position described by the depth and angles.
    /// @return Point's cartesian position, relative to its sensor's frame.
    /// @throws `std::logic_error` because I have not written this yet.
    point getPosition() const
    {
        throw std::logic_error("Sorry. I need to write a get position function here.");
        point p(std::sin(theta) * std::sin(phi), std::cos(theta) * std::sin(phi), std::cos(phi));
        return p *= depth;
    }

    /// @brief  Gets the position assuming that theta and phi follow a spherical coordinate system. 
    /// @return Point's cartesian position, relative to its sensor's frame.
    /// @note This is only really used for the `RandomLaserScanner` class. Other sensors use an image
    ///       array and not spherical notation.
    point getPositionSpherical() const {
        point p(std::sin(theta) * std::sin(phi), std::cos(theta) * std::sin(phi), std::cos(phi));
        return p *= depth;
    }
};


/// @brief Generic base class for laser depth scanner, depth cameras, and RGB-depth cameras.
template <typename T>
class BaseDepthSensor : public ForgeScanEntity
{
public:
    /// @brief Pointer to the derived class's intrinsic parameters.
    const BaseDepthSensorIntrinsics *intr;

public:
    BaseDepthSensor(const BaseDepthSensorIntrinsics& intr) :
        ForgeScanEntity(),
        intr(&intr)
        { setupDepthSensor(); }

    BaseDepthSensor(const BaseDepthSensorIntrinsics& intr, const extrinsic& extr) :
        ForgeScanEntity(extr),
        intr(&intr)
        { setupDepthSensor(); }

    BaseDepthSensor(const BaseDepthSensorIntrinsics& intr, const translation& position) :
        ForgeScanEntity(position),
        intr(&intr)
        { setupDepthSensor(); }

    BaseDepthSensor(const BaseDepthSensorIntrinsics& intr, const rotation& orientation) :
        ForgeScanEntity(orientation),
        intr(&intr)
        { setupDepthSensor(); }

    /// @brief Orients the sensor to point at the specified point.
    /// @param target Point to image. The sensor will point it's principle axis (positive Z-axis) at this point.
    /// @note If the target is equal to the current position, then the orientation is unchanged.
    void orientPrincipleAxis(point target) {
        const static Vector3d principle_axis = Eigen::Vector3d::UnitZ();
        extrinsic Tws(Eigen::Quaterniond().setFromTwoVectors(principle_axis, target - this->extr.translation()));
        this->transformBodyFrame(Tws);
    }

    /// @brief Returns the data at the sensed point.
    /// @param i Index in the storage vector to return.
    /// @return Read-only reference to the data member at `i`.
    /// @throws `std::out_of_range` If `i` is an invalid index.
    const T& data(const int& i) const { return data_vector.at(i); };

    /// @brief Returns the data at the image sensor location `(x, y)`.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Read-only reference to the data member at `(x, y)`.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    const T& data(const int& x, const int& y) const {
        throwIfInvalidSenorArray();
        return data(intr->v * x + x);
    };

    /// @brief Gets a read-only reference to the data.
    /// @return Read-only reference to the data vector.
    const std::vector<T>& data() const { return data_vector; }

    /// @brief Returns the data at the sensed point.
    /// @param i Index in the storage vector to return.
    /// @return Writable reference to the data member at `i`.
    /// @throws `std::out_of_range` If `i` is an invalid index.
    T& data(const int& i) { return data_vector.at(i); };

    /// @brief Returns the data at the image sensor location `(x, y)`.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Writable reference to the data member at `(x, y)`.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    T& data(const int& x, const int& y) {
        throwIfInvalidSenorArray();
        return data(intr->v * x + x);
    };

    /// @brief Gets a read-only reference to the data.
    /// @return Writable reference to the data vector.
    std::vector<T>& data() { return data_vector; }

    /// @brief Returns the cartesian position of the sensed point, relative to the DepthSensor's frame.
    /// @param i Index in the storage vector to return.
    /// @return Cartesian position of point `i` in the DepthSensor's reference frame.
    virtual point getPosition(const int& i) const = 0;

    /// @brief Returns the cartesian position of the sensed point at `(x, y)`, relative to the DepthSensor's frame.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Cartesian position of point `i` in the DepthSensor's reference frame.
    virtual point getPosition(const int& x, const int& y) const = 0;

    /// @brief Returns the cartesian position of all sensed points, relative to the DepthSensor's frame.
    /// @return Eigen matrix of all depth positions for the sensor, relative to the DepthSensor's reference frame.
    virtual point_list getAllPositions() const = 0;

    /// @brief Clears all sensor data, setting all values to their defaults (e.g., depth to intrinsic maximum depth).
    /// @note For derived classes where the only data is depth this is identical to calling `resetDepth`. 
    virtual void resetData() = 0;

    /// @brief Resets all depth data to the maximum depth value in the sensor's intrinsics.
    virtual void resetDepth() = 0;

protected:
    /// @brief Templated vector for storage of different collections of depth data.
    std::vector<T> data_vector;

private:
    /// @brief Setup helper function for all DepthSensor constructors.
    void setupDepthSensor() { data_vector.resize(intr->u * intr->v); }

    /// @brief Checks if the provided coordinates are valid for the image sensor.
    /// @param x X-index in the image sensor to check.
    /// @param y Y-index in the image sensor to check.
    /// @return True if valid, false else.
    bool checkValidSenorArray(const int& x, const int& y) { return (x < 0 || y < 0 || x >= intr->u || y >= intr->v ); }

    /// @brief Throws an out_of_range exception if the provided coordinates are not valid for the image sensor.
    /// @param x X-index in the image sensor to check.
    /// @param y Y-index in the image sensor to check.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    void throwIfInvalidSenorArray(const int& x, const int& y) {
        if (!checkValidSenorArray(x, y))
            throw std::out_of_range("BaseDepthSensor::throwIfInvalidSenorArray Requested coordinates were out of range.");
    }
};


/// @brief Implements a randomly-shooting laser depth sensor.
class RandomLaserScanner : public BaseDepthSensor<DepthSensorPoint>
{
public:
    /// @brief Intrinsic parameters for the RandomLaserScanner.
    const RandomLaserScannerIntrinsics intr;

public:
    RandomLaserScanner(const RandomLaserScannerIntrinsics& intr) :
        intr(intr),
        BaseDepthSensor(intr)
        { setupRandomLaserScanner(); }

    RandomLaserScanner(RandomLaserScannerIntrinsics intr, const extrinsic& extr) :
        intr(intr),
        BaseDepthSensor(intr, extr)
        { setupRandomLaserScanner(); }

    RandomLaserScanner(RandomLaserScannerIntrinsics intr, const translation& position) :
        intr(intr),
        BaseDepthSensor(intr, position)
        { setupRandomLaserScanner(); }

    RandomLaserScanner(RandomLaserScannerIntrinsics intr, const rotation& orientation) :
        intr(intr),
        BaseDepthSensor(intr, orientation)
        { setupRandomLaserScanner(); }

    /// @brief Resets depth to maximum. Samples new random laser positions.
    void resetData() final {
        std::random_device rd;
        std::uniform_real_distribution<> dist_theta(intr.theta_min, intr.theta_max);
        std::uniform_real_distribution<> dist_phi(intr.phi_min, intr.phi_max);
        std::mt19937 gen(rd());

        DepthSensorPoint p(intr.max_depth, 0, 0);
        auto generation_function = [&]() -> const DepthSensorPoint& {
            p.theta = dist_theta(gen);
            p.phi = dist_phi(gen);
            return p;
        };
        std::generate(data_vector.begin(), data_vector.end(), generation_function);
    }

    /// @brief Resets depth to maximum.
    inline void resetDepth() final { for (auto& element : data_vector) element.depth = intr.max_depth; }

    /// @brief Returns the cartesian position of the sensed point, relative to the DepthSensor's frame.
    /// @param i Index in the storage vector to return.
    /// @return Cartesian position of point `i` in the DepthSensor's reference frame.
    /// @throws `std::out_of_range` If `i` is an invalid index.
    point getPosition(const int& i) const final { return data_vector.at(i).getPositionSpherical(); }

    /// @brief Included for API consistency. A `RandomLaserScanner` has no concept of an image array, rather it is a
    ///        list of points. 
    /// @param x Index in the storage vector to return.
    /// @param y Unused.
    /// @return Cartesian position of point `x` in the DepthSensor's reference frame.
    /// @throws `std::out_of_range` If `x` is an invalid index.
    point getPosition(const int& x, const int& y) const { return getPosition(x); };

    /// @brief Returns the cartesian position of all sensed points, relative to the DepthSensor's frame.
    /// @return A dynamically allocated 3xN Eigen matrix of all depth positions for the sensor, relative to the DepthSensor's reference frame.
    point_list getAllPositions() const final
    {
        point_list sensed_points(3, intr.u * intr.v);
        for (int i = 0, n = sensed_points.cols(); i < n; ++i) {
            sensed_points.col(i) << data_vector[i].getPositionSpherical();
        }
        return sensed_points;
    }

private:
    /// @brief Setup helper function for all RandomLaserScanner constructors.
    void setupRandomLaserScanner()
    {
        resetData();
    }
};


#endif // FORGESCAN_SENSOR_READING_H
