#ifndef FORGESCAN_FORGESCAN_TYPES_H
#define FORGESCAN_FORGESCAN_TYPES_H

#include <Eigen/Geometry>


/// @brief Convenience typedef for Eigen
typedef Eigen::Vector3d Vector3d;


/// @brief Convenience typedef for Eigen; 32 bit unsigned
typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;


/// @brief Indicates a point in continuous 3D space
typedef Vector3d  point;


/// @brief A collection of points in 3D space.
typedef Eigen::Matrix3Xd point_list;


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


/// @brief Transformation matrix from the world coordinate system to an entities local coordinate system. 
typedef Eigen::Transform<double, 3, Eigen::Isometry> extrinsic;


/// @brief Translation in 3D space.
typedef Vector3d translation;


/// @brief Rotation in 3D space.
/// @note  A valid rotation matrix must be orthogonal with a determinant of 1.
typedef Eigen::Matrix3d rotation;


/// @brief Generic entity (grid, sensor, etc) that exists in 3D space.
/// @note  This essentially wraps the Eigen::Transform class. It provides useful methods for other classes to clarify
///        the semantics of coordinate transformations between one another in the code.
struct ForgeScanEntity
{
    /// @brief Extrinsic transformation from the world coordinates to the entity.
    extrinsic extr;

    /// @brief Constructs the generic entity at the world origin.
    ForgeScanEntity() { extr.setIdentity(); }

    /// @brief Constructs the generic entity.
    /// @param extr Initial pose for the entity.
    ForgeScanEntity(extrinsic extr) : extr(extr) { }

    /// @brief Constructs the generic entity at the position, no rotation.
    /// @param position Initial position for the entity.
    ForgeScanEntity(translation position)
    { 
        extr.setIdentity();
        translate(position);
    }

    /// @brief Constructs the generic entity at the world origin with the given rotation.
    /// @param orientation Initial rotation for the entity.
    ForgeScanEntity(rotation orientation)
    { 
        extr.setIdentity();
        rotateBodyFrame(orientation);
    }

    /// @brief Performs a body-frame transformation to the entity.
    /// @param other Transformation to apply.
    void transformBodyFrame(const extrinsic& other) { extr = extr * other; }

    /// @brief Performs a world-frame transformation to the entity.
    /// @param other Transformation to apply.
    void transformWorldFrame(const extrinsic& other) { extr = other * extr; }

    /// @brief Applies a translation to the entity.
    /// @param other Translation to apply.
    void translate(const translation& other) { extr.translate(other); }

    /// @brief Applies a body-frame rotation to the entity.
    /// @param other Rotation matrix to apply.
    void rotateBodyFrame(const rotation& other) { extr.rotate(other); }

    /// @brief Applies a world-frame rotation to the entity.
    /// @param other Rotation matrix to apply.
    void rotateWorldFrame(const rotation& other) { extr.prerotate(other); }

    /// @brief Generates a transformation from this entities reference frame to the other entities frame.
    /// @param other The reference frame.
    /// @return The extrinsic transformation to change points from this reference frame to the other.
    extrinsic getTransformationTo(const ForgeScanEntity& other) const { return other.extr.inverse() * this->extr; }

    /// @brief Coordinate transformation, in-place, on the provided point to shift it into this entity's frame. 
    /// @param p Point in the world frame.
    void transformFromWorldToThisFrame(point& p) const { p = extr * p.homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them into this entity's frame. 
    /// @param p_list List (matrix) of points in the world frame.
    void transformFromWorldToThisFrame(point_list& p_list) const { p_list = extr * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation on the provided point to shift it into this entity's frame.
    /// @param p Point in the world frame.
    point transformFromWorldToThisFrame(const point& p) const { return extr * p.homogeneous(); }

    /// @brief Coordinate transformation on the provided set of points to shift them into this entity's frame. 
    /// @param p_list List (Eigen::Matrix3Xd) of points in the world frame.
    point_list transformFromWorldToThisFrame(const point_list& p_list) const { return extr * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided point to shift it into the world frame. 
    /// @param p Point in this entity's frame.
    void transformFromThisToWorldFrame(point& p) const { p = extr.inverse() * p.homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them into the world frame.
    /// @param p_list List (matrix) of points in this entity's frame.
    void transformFromThisToWorldFrame(point_list& p_list) const { p_list = extr.inverse() * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation on the provided point to shift it into the world frame.
    /// @param p Point in this entity's frame.
    point transformFromThisToWorldFrame(const point& p) const { return extr.inverse() * p.homogeneous(); }

    /// @brief Coordinate transformation on the provided set of points to shift them into the world frame.
    /// @param p_list List (Eigen::Matrix3Xd) of points in this entity's frame.
    point_list transformFromThisToWorldFrame(const point_list& p_list) const { return extr.inverse() * p_list.colwise().homogeneous(); }
};


#endif // FORGESCAN_FORGESCAN_TYPES_H
