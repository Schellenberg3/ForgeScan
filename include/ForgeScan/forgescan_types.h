#ifndef FORGESCAN_FORGESCAN_TYPES_H
#define FORGESCAN_FORGESCAN_TYPES_H

#include <Eigen/Geometry>

/// Convenience typedef for Eigen
typedef Eigen::Vector3d Vector3d;

/// Convenience typedef for Eigen; 32 bit unsigned
typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;

/// Indicates a point in continuous 3D space
typedef Vector3d  point;

/// Indicates a discrete location in a 3D voxel-grid index.
typedef Vector3ui grid_idx;

/// Indicates a position in a vector list of voxels.
typedef size_t    vector_idx;


#endif // FORGESCAN_FORGESCAN_TYPES_H
