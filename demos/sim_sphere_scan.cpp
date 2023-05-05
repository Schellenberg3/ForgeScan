#include <iostream>
#include <random>
#include <cmath>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/sensor_reading.h>

#include <ForgeScanShapePrimatives/sphere.h>

using namespace ForgeScan;

static const Eigen::Vector3d WORLD_ORIGIN(0, 0, 0);
static const double GOLDEN_ANGLE_RADIANS = M_PI * (std::sqrt(5) - 1);

static std::random_device RD;
static std::mt19937 GEN(RD());
static std::uniform_real_distribution<> UNIFORM_DIST(0, 1);


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
void imageSphereLaserScanner(BaseDepthSensor<T>& sensor, const Primatives::Sphere& sphere);


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
    ForgeScan::VoxelGridProperties props(0.2);
    props.dimensions = ForgeScan::Vector3d(2, 2, 2);
    props.grid_size = ForgeScan::Vector3ui(100, 100, 100);
    props.resolution = -1;  // Let the grid size and dimensions set the resolution.
    ForgeScan::translation move(-1, -1, -1);

    ForgeScan::VoxelGrid grid(props, move);

    Primatives::Sphere sphere(0.5);

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
void imageSphereLaserScanner(BaseDepthSensor<T>& sensor, const Primatives::Sphere& sphere)
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
        if ( sphere.hit(start, end, t) ) {
            sensor.data(i).depth *= t;
        }
    }
}
