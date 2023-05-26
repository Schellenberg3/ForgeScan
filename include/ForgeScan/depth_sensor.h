#ifndef FORGESCAN_DEPTH_SENSOR_H
#define FORGESCAN_DEPTH_SENSOR_H

#include <cmath>
#include <random>
#include <stdexcept>

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/sensor_intrinsics.h>

#include <ForgeScan/Primitives/primative_geometry.h>


namespace ForgeScan {
namespace DepthSensor {


/// @brief Generic base class for laser depth scanner, depth cameras, and RGB-depth cameras.
class BaseDepthSensor : public ForgeScanEntity
{
public:
    /// @brief Pointer to the derived class's intrinsic parameters.
    const Intrinsics::BaseDepthSensor *intr;

public:
    BaseDepthSensor(const Intrinsics::BaseDepthSensor& intr) :
        ForgeScanEntity(),
        intr(&intr)
        { setupBaseDepthSensor(); }

    BaseDepthSensor(const Intrinsics::BaseDepthSensor& intr, const extrinsic& extr) :
        ForgeScanEntity(extr),
        intr(&intr)
        { setupBaseDepthSensor(); }

    BaseDepthSensor(const Intrinsics::BaseDepthSensor& intr, const translation& position) :
        ForgeScanEntity(position),
        intr(&intr)
        { setupBaseDepthSensor(); }

    BaseDepthSensor(const Intrinsics::BaseDepthSensor& intr, const rotation& orientation) :
        ForgeScanEntity(orientation),
        intr(&intr)
        { setupBaseDepthSensor(); }

    /// @brief Orients the sensor to point at the specified point.
    /// @param target Point to image. The sensor will point it's principle axis (positive Z-axis) at this point.
    /// @note If the target is equal to the current position, then the orientation is unchanged.
    void orientPrincipleAxis(point target) {
        const static Vector3d principle_axis = Eigen::Vector3d::UnitZ();
        extrinsic Tws(Eigen::Quaterniond().setFromTwoVectors(principle_axis, target - this->extr.translation()));
        this->transformBodyFrame(Tws);
    }

    /// @brief Captures a depth image of the provided PrimitiveGeometry.
    /// @param geometry The geometric primitive to image.
    /// @param reset If true, the depth sensor's depth values will be reset before the new image is calculated.
    void image(const Primitives::PrimitiveGeometry& geometry, const bool& reset = true)
    {
        /// If requested, reset all points to their maximum depth before imaging.
        if (reset) { resetDepth(); }

        /// Get the sensor's position and viewed points relative to the PrimitiveGeometry frame.
        /// This is needed for the fast AABB checks performed on each ray.
        point start = extr.translation();   // World frame
        geometry.fromWorldToThis(start);    // Primitive frame

        point_list end_points = getAllPositions();     // Sensor frame
        geometry.fromOtherToThis(*this, end_points);   // Primitive frame

        int i = 0;
        double t = 1;
        for (auto& depth : depth_vector)
        {   /// Run intersection search; if there is a valid hit then we scale the depth value by t, the returned time.
            /// All we do is scale the depth value.
            if ( geometry.hit(start, end_points.col(i), t) ) {
                depth *= t;
            }
            ++i;
        }
    }

    /// @brief Captures a depth image of a scene consisting of multiple PrimitiveGeometry objects.
    /// @param scene A collection of GeometricPrimitives to image.
    /// @param reset If true, the depth sensor's depth values will be reset before the new image is calculated.
    void image(const Primitives::Scene& scene, const bool& reset = true)
    {
        /// If requested, reset all points to their maximum depth before imaging.
        if (reset) { resetDepth(); }
        for (const auto& geometry : scene)
            image(*geometry, false);
    }

    /// @brief Returns the depth at the sensed point, checking validity.
    /// @param i Index in the storage vector to return.
    /// @return Read-only reference to the depth member at `i`.
    /// @throws `std::out_of_range` If `i` is an invalid index.
    const double& at(const size_t& i) const { return depth_vector.at(i);}

    /// @brief Returns the depth at the image sensor location `(x, y)`, checking validity.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Read-only reference to the depth member at `(x, y)`.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    const double& at(const size_t& x, const size_t& y) const { return at( throw_xy_to_i(x, y) ); }

    /// @brief Returns the depth at the sensed point, checking validity.
    /// @param i Index in the storage vector to return.
    /// @return Writable reference to the depth member at `i`.
    /// @throws `std::out_of_range` If `i` is an invalid index.
    double& at(const size_t& i) { return depth_vector.at(i); }

    /// @brief Returns the depth at the image sensor location `(x, y)`, checking validity.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Writable reference to the depth member at `(x, y)`.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    double& at(const size_t& x, const size_t& y) { return at( throw_xy_to_i(x, y) ); }

    /// @brief Returns the depth at the sensed point. Unchecked access.
    /// @param i Index in the storage vector to return.
    /// @return Read-only reference to the depth member at `i`.
    const double& operator()(const size_t& i) const { return depth_vector[i]; }

    /// @brief Returns the depth at the image sensor location `(x, y)`. Unchecked access.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Read-only reference to the depth member at `(x, y)`.
    const double& operator()(const size_t& x, const size_t& y) const { return operator()( xy_to_i(x, y) ); }

    /// @brief Returns the depth at the sensed point. Unchecked access.
    /// @param i Index in the storage vector to return.
    /// @return Writable reference to the depth member at `i`.
    double& operator()(const size_t& i) { return depth_vector[i]; }

    /// @brief Returns the depth at the image sensor location `(x, y)`. Unchecked access.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Writable reference to the depth member at `(x, y)`.
    double& operator()(const size_t& x, const size_t& y) { return operator()( xy_to_i(x, y) ); }

    /// @brief Gets a read-only reference to the depth.
    /// @return Read-only reference to the depth vector.
    const std::vector<double>& depth() const { return depth_vector; }

    /// @brief Gets a writable reference to the depth vector.
    /// @return Writable reference to the depth vector.
    std::vector<double>& depth() { return depth_vector; }

    /// @brief Moves the camera to a random pose based on a spherical.
    /// @param center Where to point the sensor's principle axis.
    /// @param radius Distance between the center and the sensor.
    /// @note Uses proper uniform random sampling from a sphere, see: http://corysimon.github.io/articles/uniformdistn-on-sphere/
    void setPoseRandom(const Vector3d& center, const double& radius)
    {
        double theta = 2 * M_PI * uniform_dist(gen);        /// 0 - 360 degrees in theta  (angle around the positive X-axis)
        double phi = std::acos(1 - 2 * uniform_dist(gen));  /// 0 - 180 degrees in phi    (angle from positive Z-axis)
        if (uniform_dist(gen) < 0.5) phi *= -1;             /// -180 - 180 degrees in phi (important for covering all camera orientations)

        point position(std::sin(theta) * std::sin(phi), std::cos(theta) * std::sin(phi), std::cos(phi));
        position *= radius;
        position += center;

        extr.setIdentity();
        translate(position);
        orientPrincipleAxis(center);
    }

    /// @brief Moves the sensor to a new pose with uniform spacing around the specified center point.
    /// @param center Where to point the sensor's principle axis.
    /// @param radius Radius from world origin to place the sensor at.
    /// @param view_number Which uniformly sampled view to return. Range is from 0 to (N - 1).
    /// @param total_views Number of uniform samples to select the view from.
    /// @note Base on Fibonacci sphere sequence, see: https://stackoverflow.com/questions/9600801/
    /// @throws `std::invalid_argument` if total_views is less than 1.
    /// @throws `std::invalid_argument` if view_number is not in the range: 0 <= view_number < total_views
    void setPoseUniform(const Vector3d& center, const double& radius, const int& view_number, const int& total_views)
    {
        static const double GOLDEN_ANGLE_RADIANS = M_PI * (std::sqrt(5) - 1);
        if (total_views < 1) throw std::invalid_argument("Cannot have less than 1 total view.");
        if (total_views < view_number || view_number < 0)
            throw std::invalid_argument("For a valid result view number must be in the range: 0 <= view_number < total_views");

        /// Y walks from 1 to -1
        double y = 1 - (view_number / (float)(total_views - 1)) * 2;

        /// Radius of the sphere at that location of y.
        double r_y = std::sqrt(1 - y*y);

        /// Calculate the proper angles. Phi moves from 0 to 180 as y decrements.
        double phi = std::acos(y);
        double theta = GOLDEN_ANGLE_RADIANS * view_number;
        
        /// Solve the last two distances. 
        double x = std::cos(theta) * r_y;
        double z = std::sin(theta) * r_y;

        point position(x, y, z);
        position *= radius;
        position += center;

        extr.setIdentity();
        translate(position);
        orientPrincipleAxis(center);
    }

    /// @brief Returns the cartesian position of the sensed point, relative to the DepthSensor's frame.
    /// @param i Index in the storage vector to return.
    /// @return Cartesian position of point `i` in the DepthSensor's reference frame.
    /// @throws `std::out_of_range` If `i` is an invalid index.
    virtual point getPosition(const size_t& i) const 
    {
        throwIfInvalidSenorArray(i);
        return getPosition(i, i % intr->u, i / intr->u);
    }

    /// @brief Returns the cartesian position of the sensed point at `(x, y)`, relative to the DepthSensor's frame.
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Cartesian position of point at `(x, y)` in the DepthSensor's reference frame.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    virtual point getPosition(const size_t& x, const size_t& y) const
    {
        return getPosition(throw_xy_to_i(x, y), x, y);
    }

    /// @brief Returns the cartesian position of all sensed points, relative to the DepthSensor's frame.
    /// @return Eigen matrix of all depth positions for the sensor, relative to the DepthSensor's reference frame.
    virtual point_list getAllPositions() const
    {
        const double d_theta = intr->fov_y() / (intr->v - 1);
        const double d_phi   = intr->fov_x() / (intr->u - 1);

        point_list sensed_points(3, intr->size());

        int i = 0;
        double theta = intr->theta_max, phi = intr->phi_max;
        for (int y = 0; y < intr->v; ++y)
        {
            phi = intr->phi_max;
            for (int x = 0; x < intr->u; ++x)
            {
                /// TODO: Improve by re-implementing rotationToCartesian to minimize trig function calls on theta
                ///       when we are iterating in this loop and not arbitrarily calling points.
                sensed_points.col(i) << rotationToCartesian(depth_vector[i], theta, phi);
                phi -= d_phi;
                ++i;
            }
            theta -= d_theta;
        }
        return sensed_points;
    }

    /// @brief Resets all depth depth to the maximum depth value in the sensor's intrinsics.
    virtual void resetDepth() = 0;

protected:
    /// @brief Templated vector for storage of different collections of depth depth.
    std::vector<double> depth_vector;

protected:
    /// @brief Turns the 2D-coordinates into a vector index.
    size_t xy_to_i(const size_t& x, const size_t& y) const { return intr->u * y + x; }

    /// @brief Turns the 2D-coordinates into a vector index. Checks validity of returned result.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    size_t throw_xy_to_i(const size_t& x, const size_t& y) const {
        throwIfInvalidSenorArray(x, y);
        return xy_to_i(x, y);
    }

    /// @brief Checks if the provided coordinates are valid for the image sensor.
    /// @param x X-index in the image sensor to check.
    /// @param y Y-index in the image sensor to check.
    /// @return True if valid, false else.
    bool checkValidSenorArray(const size_t& x, const size_t& y) const {
        /// Negative integers cast to size_t should necessarily be greater than any reasonable intrinsics dimensions.
        return (x < intr->u && y < intr->v);
    }

    /// @brief Checks if the provided coordinates are valid for the image sensor.
    /// @param i Index in the depth vector to check.
    /// @return True if valid, false else.
    bool checkValidSenorArray(const size_t& i) const { 
        /// Negative integers cast to size_t should necessarily be greater than any reasonable intrinsics dimensions.
        return (i < depth_vector.size());
    }

    /// @brief Throws an out_of_range exception if the provided coordinates are not valid for the image sensor.
    /// @param x X-index in the image sensor to check.
    /// @param y Y-index in the image sensor to check.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    void throwIfInvalidSenorArray(const size_t& x, const size_t& y) const {
        if (!checkValidSenorArray(x, y))
            throw std::out_of_range("BaseDepthSensor::throwIfInvalidSenorArray Requested coordinates were out of range.");
    }

    /// @brief Throws an out_of_range exception if the provided coordinates are not valid for the image sensor.
    /// @param i Index in the depth vector to check.
    /// @throws `std::out_of_range` If `(x, y)` is an invalid index.
    void throwIfInvalidSenorArray(const size_t& i) const {
        if (!checkValidSenorArray(i))
            throw std::out_of_range("BaseDepthSensor::throwIfInvalidSenorArray Requested coordinates were out of range.");
    }

    void resetDepthLaserScanner() { std::fill(depth_vector.begin(), depth_vector.end(), intr->d_max); }

    void resetDepthDepthCamera()
    {
        const double d_theta = intr->fov_y() / (intr->v - 1);
        const double d_phi   = intr->fov_x() / (intr->u - 1);

        int i = 0;
        double theta = intr->theta_max, phi = intr->phi_max;
        double cos_theta = 0;

        /// TODO: Exploit symmetry.
        for (int y = 0; y < intr->v; ++y)
        {
            phi = intr->phi_max;
            cos_theta = std::cos(theta);
            for (int x = 0; x < intr->u; ++x)
            {
                depth_vector[i] = intr->d_max / (cos_theta * std::cos(phi));
                phi -= d_phi;
                ++i;
            }
            theta -= d_theta;
        }
    }

private:
    /// @brief Random number engine for performing sampling on the uniform real distribution.
    std::mt19937 gen;
    
    /// @brief Uniform distribution over [0, 1).
    std::uniform_real_distribution<double> uniform_dist;

private:
    /// @brief Setup helper function for all DepthSensor constructors. Initialized all values to positive infinity.
    void setupBaseDepthSensor() {
        depth_vector.resize(intr->size(), std::numeric_limits<float>::infinity());

        uniform_dist = std::uniform_real_distribution<double>(0.0, 1.0);

        std::random_device rd;     // Used to obtain a seed for the random number engine.
        gen = std::mt19937(rd());  // Standard mersenne_twister_engine seeded with the random device seed.
    }

    /// @brief Returns the cartesian position of the sensed point at `(x, y)`, relative to the DepthSensor's frame.
    /// @details This private implementation is called by either public one. It trusts that the `(x, y)` or `i` index
    ///          are both valid. This prevents either public function from re-calculating the index or array coordinates. 
    /// @param i Index in the depth vector for the position (x, y).
    /// @param x X-index in the image sensor to return.
    /// @param y Y-index in the image sensor to return.
    /// @return Cartesian position of point at `(x, y)` in the DepthSensor's reference frame.
    virtual point getPosition(const size_t& i, const size_t& x, const size_t& y) const
    {
        double theta = intr->theta_max - y * (intr->fov_y() / (intr->v - 1));
        double phi   = intr->phi_max   - x * (intr->fov_x() / (intr->u - 1));
        // double phi   = intr->phi_min   + x * (intr->fov_x() / (intr->u - 1));
        return rotationToCartesian(depth_vector[i], theta, phi);
    }

    /// @brief  Calculates cartesian position when theta and phi represent rotations about the X-axis then the Y-axis.
    /// @return Cartesian position relative to the sensor's reference frame. 
    static point rotationToCartesian(const double& depth, const double& theta, const double& phi)
    {
        /// Rx(theta) * Ry(phi) -- This rotation prevents mirroring which occurs if we rotate in the other order.
        point p(std::sin(phi), -std::sin(theta) * std::cos(phi), std::cos(theta) * std::cos(phi));
        p *= depth;
        return p;
    }
};


class DepthCamera : public BaseDepthSensor
{
public:
    DepthCamera(const Intrinsics::DepthCamera& intr) :
        BaseDepthSensor(intr)
        { setup(); }

    DepthCamera(const Intrinsics::DepthCamera& intr, const extrinsic& extr) :
        BaseDepthSensor(intr, extr)
        { setup(); }

    DepthCamera(const Intrinsics::DepthCamera& intr, const translation& position) :
        BaseDepthSensor(intr, position)
        { setup(); }

    DepthCamera(const Intrinsics::DepthCamera& intr, const rotation& orientation) :
        BaseDepthSensor(intr, orientation)
        { setup(); }

    void resetDepth() override final { resetDepthDepthCamera(); }

private:
    void setup() { resetDepthDepthCamera(); }
};


class LaserScanner : public BaseDepthSensor
{
public:
    LaserScanner(const Intrinsics::LaserScanner& intr) :
        BaseDepthSensor(intr)
        { setup(); }

    LaserScanner(const Intrinsics::LaserScanner& intr, const extrinsic& extr) :
        BaseDepthSensor(intr, extr)
        { setup(); }

    LaserScanner(const Intrinsics::LaserScanner& intr, const translation& position) :
        BaseDepthSensor(intr, position)
        { setup(); }

    LaserScanner(const Intrinsics::LaserScanner& intr, const rotation& orientation) :
        BaseDepthSensor(intr, orientation)
        { setup(); }

    void resetDepth() override final { resetDepthLaserScanner(); }

private:
    void setup() { resetDepthLaserScanner(); }
};


} // DepthSensor
} // ForgeScan

#endif // FORGESCAN_DEPTH_SENSOR_H
