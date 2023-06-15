#ifndef FORGESCAN_TYPES_H
#define FORGESCAN_TYPES_H

#include <Eigen/Geometry>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


namespace ForgeScan {


/// @brief Convenience typedef for Eigen; 32 bit unsigned vector.
typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;

/// @brief Indicates a discrete location in a 3D voxel-grid index.
typedef Vector3ui index;

/// @brief Convenience typedef for Eigen
typedef Eigen::Vector3d Vector3d;

/// @brief Indicates a point in continuous 3D space
typedef Vector3d point;

/// @brief A collection of points in 3D space.
typedef Eigen::Matrix3Xd point_list;

/// @brief Counts number of updates performed on a Voxel. A max count of 65,535 using 2 Bytes.
typedef uint16_t update_count;

/// @brief Counts number of views that have updated a Voxel. Uses 15 bytes to count, leaving a
///        a max count of 32,767 using 2 Bytes. The MSB is reserved as a flag to update this value once
///        all rays from a view have been cast.
typedef uint16_t view_count;

/// @brief A distance value stored in a Voxel. Negative values indicate the voxel is behind the
///        surface while positive ones indicate it is in from of the surface; the surface is implicitly
///        represented by the zero-level of the Grid.
typedef float voxel_dist;

/// @brief Centrality score for a Voxel. Score decreases to zero at the edge of the sensor's FOV and is
///        close to to 1 at the sensor's principle axis.
typedef float centrality;

/// @brief Normality score for a Voxel. Score decreases to zero when a ray direction is perpendicular
///        to the estimated surface normal at a point and is close to one when these vectors are parallel.
typedef float normality;

/// @brief Density score for the voxel. Score is increases with the number if points that fall in that voxel
///        in a given view.
typedef float density;

/// @brief Transformation matrix from the world coordinate system to an entity's local coordinate system.
typedef Eigen::Transform<double, 3, Eigen::Isometry> extrinsic;

/// @brief Translation in 3D space.
typedef Vector3d translation;

/// @brief Rotation in 3D space.
/// @note  A valid rotation matrix must be orthogonal with a determinant of 1.
typedef Eigen::Matrix3d rotation;


} // namespace ForgeScan

#endif // FORGESCAN_TYPES_H
