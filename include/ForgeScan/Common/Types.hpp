#ifndef FORGE_SCAN_COMMON_TYPES_HPP
#define FORGE_SCAN_COMMON_TYPES_HPP

#include <vector>

#include <Eigen/Geometry>


namespace forge_scan {


// *************************************** GENERAL TYPES *************************************** //


/// @brief Represents an image taken by a depth camera.
/// @note Can be mapped to/from OpenCV. See: https://stackoverflow.com/a/21706778
typedef Eigen::MatrixXf DepthImage;


/// @brief Index of a voxel within a Grid.
typedef Eigen::Matrix<size_t, 3, 1> Index;


/// @brief Number of voxels in a Grid.
typedef Eigen::Matrix<size_t, 3, 1> GridSize;



// ********************************** EIGEN / GEOMETRY TYPES *********************************** //


/// @brief Transformation matrix from the world coordinate system to an entity's local coordinate system.
typedef Eigen::Transform<float, 3, Eigen::Isometry> Extrinsic;


/// @brief Collection of location in 3D space in the same reference frame. Shaped 3xN.
typedef Eigen::Matrix3Xf PointMatrix;


/// @brief Location in 3D space.
typedef Eigen::Vector3f Point;


/// @brief Translation in 3D space.
typedef Eigen::Vector3f Translation;


/// @brief Vector in 3D space.
typedef Eigen::Vector3f Ray;


/// @brief Vector in 3D space, expected to be of unit length.
typedef Eigen::Vector3f Direction;


/// @brief Rotation in 3D space.
/// @note  A valid Rotation matrix must be orthogonal with a determinant of 1.
typedef Eigen::Matrix3f Rotation;


} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_TYPES_HPP
