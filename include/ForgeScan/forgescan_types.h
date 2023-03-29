#ifndef FORGESCAN_FORGESCAN_TYPES_H
#define FORGESCAN_FORGESCAN_TYPES_H

#include <Eigen/Geometry>


/// @brief Convenience typedef for Eigen
typedef Eigen::Vector3d Vector3d;


/// @brief Convenience typedef for Eigen; 32 bit unsigned
typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;


/// @brief Indicates a point in continuous 3D space
typedef Vector3d  point;


/// @brief Indicates a discrete location in a 3D voxel-grid index.
typedef Vector3ui grid_idx;


/// @brief Indicates a position in a vector list of voxels.
typedef size_t vector_idx;


/// @brief Counts number of updates performed on a VoxelElement. A max count of 65,535 using 2 Bytes.
typedef uint16_t update_count;


/// @brief Counts number of views that have updated a VoxelElement. Uses 15 bytes to count, leaving a
///        a max count of 32,767 using 2 Bytes. The MSB is reserved as a flag to update this value once
///        all rays from a view have been cast.
typedef uint16_t view_count;


/// @brief A distance value stored in a VoxelElement. Negative values indicate the voxel is behind the
///        surface while positive ones indicate it is in fron of the surface; the surface is implicitly
///        represented by the zero-level of the VoxelGrid.
typedef float voxel_dist;


/// @brief Centrality score for a VoxelElement. Score decreases to zero at the edge of the sensor's FOV and is
///        close to to 1 at the sensor's principle axis.
typedef float centrality;


/// @brief Normality score for a VoxelElement. Score decreases to zere when a ray direction is perpendicular
///        to the estimated surface normal at a point and is close to one when these vectors are parallel.
typedef float normality;


/// @brief Density score for the voxel. Score is increases with the number if points that fall in that voxel
///        in a given view.
typedef float density;

#endif // FORGESCAN_FORGESCAN_TYPES_H
