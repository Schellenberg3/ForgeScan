#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/sensor_reading.h>

#include <iostream>
#include <random>
#include <cmath>


static const Eigen::Vector3d WORLD_ORIGIN(0, 0, 0);
static const double GOLDEN_ANGLE_RADIANS = M_PI * (std::sqrt(5) - 1);

static std::random_device RD;
static std::mt19937 GEN(RD());
static std::uniform_real_distribution<> UNIFORM_DIST(0, 1);

/// @brief A simple analytical sphere object.
struct Sphere
{
    /// @brief Sphere center in world space coordinates
    Eigen::Vector3d center;

    /// @brief Sphere radius in world units.
    double radius;

    /// @brief Constructs an analytical sphere with the given radius at the specified position
    /// @param radius Radius value for the sphere. Default is 1 unit.
    /// @param center Location of the sphere's center point in world coordinate.
    ///               Default places the sphere at the origin
    /// @note This takes the absolute value of the provided radius value.
    Sphere(double radius = 1, Eigen::Vector3d center = Eigen::Vector3d::Zero()) :
        radius(std::abs(radius)), center(center) { }
};


/// @brief Generates new camera pose view which faces the origin by randomly sampling points.
/// @tparam T Templated on the datatype of the depth sensor. Provided by the derived type. 
/// @param sensor Sensor to generate and apply a new pose to.
/// @param radius Radius from world origin to place the sensor at.
/// @note Uses proper uniform random sampling from a sphere, see: http://corysimon.github.io/articles/uniformdistn-on-sphere/
template <typename T>
void getSensorPoseRandom(BaseDepthSensor<T>& sensor, const double& radius);


/// @brief Generates a new sensor pose which faces the origin in an ordered, uniform manner.
/// @tparam T Templated on the datatype of the depth sensor. Provided by the derived type. 
/// @param sensor Sensor to generate and apply a new pose to.
/// @param radius Radius from world origin to place the sensor at.
/// @param view_number Which uniformly sampled view to return. Range is from 0 to (N - 1).
/// @param total_views Number of uniform samples to select the view from.
/// @note Base on Fibonacci sphere sequence, see: https://stackoverflow.com/questions/9600801/
/// @throws `std::invalid_argument` if total_views is less than 1.
/// @throws `std::invalid_argument` if view_number is not in the range: 0 <= view_number < total_views
template <typename T>
void getSensorPoseUniform(BaseDepthSensor<T>& sensor, const double& radius, const int& view_number, const int& total_views);


/// @brief Images the provided sphere with the given depth sensor.
/// @tparam T Templated on the datatype of the depth sensor. Provided by the derived type. 
/// @param sensor Depth sensor to store results in.
/// @param sphere Sphere to check intersections against.
template <typename T>
void imageSphereLaserScanner(BaseDepthSensor<T>& sensor, const Sphere& sphere);


/// @brief Determines if, and where, the line between the start and end points first intersects the provided sphere.
/// @param sphere Sphere to check intersections against.
/// @param start  Start point (position on the line when t = 0).
/// @param end    End point (position on the line when t = 1).
/// @param t      Intersection time (output variable). Values 0 <= t <= 1 are valid on the line segment.
///                - If the line DOES NOT intersect we return false with t unchanged.
///                - If it DOES intersect but NOT inside the bounds, then we return false with t set to the minimum (in magnitude)
///                  intersection time, even if this is not on the segment.
/// @return True if the line intersects and does so in a valid region of the line.
/// @note Find the intersection point with: `intersection = start + (end - start) * t`.
/// @note This preferences the first positive intersection: if the line intersects at both a positive and negative t the
///       positive one is returned, ever if its magnitude is greater. If both intersection times are negative then the
///       one closest to zero is returned. This covers cases where the camera is inside the sphere.
bool lineIntersectsSphere(const Sphere& sphere, const Vector3d& start, const Vector3d& end, double& t);


/// @brief Simulates the scanning of a sphere by a narrow FOV laser scanner.
/// @param num_pts   Positive, integer number of points to add to the sensor. Default 1000.
/// @param num_view  Positive, integer number of views to add. Default 10.
/// @param first_pos If true will set the first view to be fixed down the world +Z axis. Default `true`.
/// @param random_pose If included will randomly sample new poses. Be default poses are evenly distributed.
int main(int argc, char* argv[])
{
    int num_pts = 1000, num_view = 10;
    bool first_pos = true, random_pose = false;
    double camera_radius = 2.5;
    if (argc >= 2) num_pts  = std::abs( std::stoi(argv[1]) );  // Set the number of sensor points
    if (argc >= 3) num_view = std::abs( std::stoi(argv[2]) );  // Set the number of sensors
    if (argc >= 4) first_pos = (argv[3] == "true");            // If true, sets first view to be fixed down the world +Z axis
    if (argc >= 5) random_pose = !random_pose;                 // If anything is passed for argv[4], we will use a random sampling strategy
    std::cout << "Running for " << num_view << " sensors with " << num_pts << " samples each." << std::endl;

    // Set up the VoxelGrid as a 2m x 2m x 2m cube with 0.02 m resolution
    VoxelGridProperties props(0.2);
    props.dimensions = Vector3d(2, 2, 2);
    props.grid_size = Vector3ui(100, 100, 100);
    props.resolution = -1;  // Let the grid size and dimensions set the resolution.
    translation move(-1, -1, -1);

    VoxelGrid grid(props, move);

    Sphere sphere(0.5);

    RandomLaserScannerIntrinsics scanner_intr(100, num_pts, 0.5 * M_PI_4);
    RandomLaserScanner laser_scanner(scanner_intr);

    if (first_pos)
    {
        laser_scanner.translate(point(0, 0, camera_radius));
        laser_scanner.orientPrincipleAxis(WORLD_ORIGIN);

        imageSphereLaserScanner(laser_scanner, sphere);
        grid.addSensor(laser_scanner);

        std::cout << "Added deterministic first view." << std::endl;
        --num_view;  // Decrement the number of requested views for the rest of the program
    }

    std::string strategy = random_pose ? "random" : "uniform";
    std::cout << "Adding " << num_view << " new views. Using a " + strategy + " strategy" << std::endl;
    for (int i = 0; i < num_view; ++i)
    {
        if (random_pose) {
            getSensorPoseRandom(laser_scanner, camera_radius);
        } else {
            getSensorPoseUniform(laser_scanner, camera_radius, i, num_view);
        }
        imageSphereLaserScanner(laser_scanner, sphere);
        grid.addSensor(laser_scanner);
    }
    std::string fname = "sim_sphere_scan";
    grid.saveXDMF(fname);
    std::cout << "Complete! Saved file as: " + fname << std::endl;
    return 0;
}


template <typename T>
void getSensorPoseRandom(BaseDepthSensor<T>& sensor, const double& radius)
{
    double theta = 2 * M_PI * UNIFORM_DIST(GEN);        /// 0 - 360 degrees in theta  (angle around the positive X-axis)
    double phi = std::acos(1 - 2 * UNIFORM_DIST(GEN));  /// 0 - 180 degrees in phi    (angle from positive Z-axis)
    if (UNIFORM_DIST(GEN) < 0.5) phi *= -1;             /// -180 - 180 degrees in phi (important for covering all camera orientations)

    point position(std::sin(theta) * std::sin(phi),
                   std::cos(theta) * std::sin(phi),
                   std::cos(phi));
    position *= radius;

    sensor.extr.setIdentity();
    sensor.translate(position);
    sensor.orientPrincipleAxis(WORLD_ORIGIN);
}


template <typename T>
void getSensorPoseUniform(BaseDepthSensor<T>& sensor, const double& radius, const int& view_number, const int& total_views)
{
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

    sensor.extr.setIdentity();
    sensor.translate(position);
    sensor.orientPrincipleAxis(WORLD_ORIGIN);
}


template <typename T>
void imageSphereLaserScanner(BaseDepthSensor<T>& sensor, const Sphere& sphere)
{
    /// Star position is sensor position in the world frame.
    const Vector3d start = sensor.extr.translation();
    
    /// End position is the sensed point at an index. Transformed to the world frame.
    Eigen::Vector3d end(0, 0, 0);
    
    /// Set all points to max_depth before imaging.
    sensor.resetDepth();

    double t = 1;
    for (int i = 0, num_pts = sensor.intr->size(); i < num_pts; ++i)
    {
        end = sensor.toWorldFromThis( sensor.getPosition(i) );

        /// Run intersection search. Both start and end are in the world frame. And the value t is just a proportion
        /// and is not attached to any frame. Thus, we scale the depth by this if we have a valid intersection.
        if ( lineIntersectsSphere(sphere, start, end, t) ) {
            sensor.data(i).depth *= t;
        }
    }
}


bool lineIntersectsSphere(const Sphere& sphere, const Vector3d& start, const Vector3d& end, double& t)
{
    /// Adapted from https://stackoverflow.com/questions/6533856 with a partial quadratic solver for only the
    /// real-valued solutions of the intersection. Also, see; http://paulbourke.net/geometry/circlesphere/

    /// Quadratic equation for intersection:
    ///     0 = A*(x*x) + B*x + C
    double R2 = sphere.radius * sphere.radius;
    double A = ((start - end).array().pow(2)).sum();
    double C = ((start - sphere.center).array().pow(2)).sum() - R2;
    double B = ((end - sphere.center).array().pow(2)).sum() - A - C - R2;

    /// Find quadratic equation determinant. Early exit if negative (complex solutions mean no intersection).
    double D = B*B - 4*A*C;
    if (D < 0) return false;

    /// Pre-calculations help us optimize the quadratic formula. And checking the sign of B lets
    /// us utilize an numerically stable form in which only addition OR subtraction is  required.
    ///     D = sqrt(B*B - 4*A*C)
    ///     if B < 0 
    ///         X_1 = (-B + D) / 2*A
    ///         X_2 = 2*C / (-B + D)
    ///         (Leads to adding two positives)
    ///     if B >= 0
    ///         X_1 = (-B - D) / 2*A
    ///         X_2 = 2*C / (-B - D)
    /// In short, the first case lets us add two positives and the second lets us subtract two negatives. This
    /// is ideal as it avoids any case where we subtract quantities with the same sign. In cases where these values
    /// are similar in magnitude (for this case, when 4*A*C is small) this leads to imprecision in rounding.
    /// For details on the numeric stability see: https://people.csail.mit.edu/bkph/articles/Quadratics.pdf
    
    /// Both cases require the following values which we may pre-compute 
    D = std::sqrt(D);
    A *= 2;
    C *= 2;
    B *= -1;

    if (B > 0) {
        B += D;
    } else {
        B -= D;
    }
    t = C / B;
    double x = B / A;

    if (t >= 0) {
        /// Find minimum if both are positive. Else, leave t unchanged as the other solution is non-positive.
        if (x > 0) t = std::min(t, x);
    } else if (t <= 0) {
        /// Find maximum if both are negative. Else, set t to the other solution, which must be non-negative.
        t = x < 0 ? std::max(t, x) : x;
    }
    /// For the case t == 0 the answers are the same so no comparison is needed.

    /// In this, 0 <= t <= 1 indicates a point between the two values of interest.
    return ( 0 <= t && t <= 1 );
}
