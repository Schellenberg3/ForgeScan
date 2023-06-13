#ifndef FORGESCAN_ENTITY_H
#define FORGESCAN_ENTITY_H

#include "ForgeScan/types.h"


namespace ForgeScan {


/// @brief Generic entity (grid, sensor, etc) that exists in 3D space.
/// @note  This essentially wraps the Eigen::Transform class. It provides useful methods for other classes to clarify
///        the semantics of coordinate transformations between one another in the code.
struct Entity
{
    /// @brief Extrinsic transformation from the world coordinates to the entity.
    extrinsic extr;

    /// @brief Constructs the generic entity at the world origin.
    Entity() { extr.setIdentity(); }

    /// @brief Constructs the generic entity.
    /// @param extr Initial pose for the entity.
    Entity(const extrinsic& extr) : extr(extr) { }

    /// @brief Constructs the generic entity at the position, no rotation.
    /// @param position Initial position for the entity.
    Entity(const translation& position)
    {
        extr.setIdentity();
        translate(position);
    }

    /// @brief Constructs the generic entity at the world origin with the given rotation.
    /// @param orientation Initial rotation for the entity.
    Entity(const rotation& orientation)
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

    /// @brief Generates a transformation from this entity's reference frame to the other entity's frame.
    /// @param other The reference frame.
    /// @return The extrinsic transformation to change points from this reference frame to the other.
    extrinsic getTransformationTo(const Entity& other) const { return other.extr.inverse() * this->extr; }

    /// @brief Generates a transformation from another entity's reference to this entity's reference frame.
    /// @param other The reference frame.
    /// @return The extrinsic transformation to change points from this reference frame to the other.
    extrinsic getTransformationFrom(const Entity& other) const { return this->extr.inverse() * other.extr; }

    /// @brief Coordinate transformation, in-place, on the provided point to shift it into this entity's frame.
    /// @param p Point in the world frame.
    void toThisFromWorld(point& p) const { p = extr.inverse() * p.homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them into this entity's frame.
    /// @param p_list List (a 3xN matrix) of points in the world frame.
    void toThisFromWorld(point_list& p_list) const { p_list = extr.inverse() * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation on the provided point to shift it into this entity's frame.
    /// @param p Point in the world frame.
    point toThisFromWorld(const point& p) const { return extr.inverse() * p.homogeneous(); }

    /// @brief Coordinate transformation on the provided set of points to shift them into this entity's frame.
    /// @param p_list List (a 3xN matrix) of points in the world frame.
    point_list toThisFromWorld(const point_list& p_list) const { return extr.inverse() * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided point to shift it into the world frame.
    /// @param p Point in this entity's frame.
    void toWorldFromThis(point& p) const { p = extr * p.homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them into the world frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    void toWorldFromThis(point_list& p_list) const { p_list = extr * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation on the provided point to shift it into the world frame.
    /// @param p Point in this entity's frame.
    point toWorldFromThis(const point& p) const { return extr * p.homogeneous(); }

    /// @brief Coordinate transformation on the provided set of points to shift them into the world frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    point_list toWorldFromThis(const point_list& p_list) const { return extr * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided point to shift it into the other entity's frame.
    /// @param p Point in this entity's frame.
    /// @param other Other ForgeScan entity.
    void toOtherFromThis(point& p, const Entity& other) const
        { return toOtherFromThis(p, other.extr); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them into the other entity's frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param other Other ForgeScan entity.
    void toOtherFromThis(point_list& p_list, const Entity& other) const
        { return toOtherFromThis(p_list, other.extr); }

    /// @brief Coordinate transformation on the provided point to shift it into the other entity's frame.
    /// @param p Point in this entity's frame.
    /// @param other Other ForgeScan entity.
    point toOtherFromThis(const point& p, const Entity& other) const
        { return toOtherFromThis(p, other.extr); }

    /// @brief Coordinate transformation on the provided set of points to shift them into the other entity's frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param other Other ForgeScan entity.
    point_list toOtherFromThis(const point_list& p_list, const Entity& other) const
        { return toOtherFromThis(p_list, other.extr); }

    /// @brief Coordinate transformation, in-place, on the provided point to shift it into the other entity's frame.
    /// @param p Point in this entity's frame.
    /// @param extr Other reference frame.
    void toOtherFromThis(point& p, const extrinsic& extr) const
        { p = (extr.inverse() * this->extr) * p.homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them into the other entity's frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param extr Other reference frame.
    void toOtherFromThis(point_list& p_list, const extrinsic& extr) const
        { p_list = (extr.inverse() * this->extr) * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation on the provided point to shift it into the other entity's frame.
    /// @param p Point in this entity's frame.
    /// @param extr Other reference frame.
    point toOtherFromThis(const point& p, const extrinsic& extr) const
        { return (extr.inverse() * this->extr) * p.homogeneous(); }

    /// @brief Coordinate transformation on the provided set of points to shift them into the other entity's frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param extr Other reference frame.
    point_list toOtherFromThis(const point_list& p_list, const extrinsic& extr) const
        { return (extr.inverse() * this->extr) * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided point to shift it into the other entity's frame.
    /// @param p Point in this entity's frame.
    /// @param other Other ForgeScan entity.
    void toThisFromOther(point& p, const Entity& other) const
        { return toThisFromOther(p, other.extr); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them into the other entity's frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param other Other ForgeScan entity.
    void toThisFromOther(point_list& p_list, const Entity& other) const
        { return toThisFromOther(p_list, other.extr); }

    /// @brief Coordinate transformation on the provided point to shift it into the other entity's frame.
    /// @param p Point in this entity's frame.
    /// @param other Other ForgeScan entity.
    point toThisFromOther(const point& p, const Entity& other) const
        { return toThisFromOther(p, other.extr); }

    /// @brief Coordinate transformation on the provided set of points to shift them into the other entity's frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param other Other ForgeScan entity.
    point_list toThisFromOther(const point_list& p_list, const Entity& other) const
        { return toThisFromOther(p_list, other.extr); }

        /// @brief Coordinate transformation, in-place, on the provided point to shift it to the provided reference frame.
    /// @param p Point in this entity's frame.
    /// @param extr Other reference frame.
    void toThisFromOther(point& p, const extrinsic& extr) const
        { p = (this->extr.inverse() * extr) * p.homogeneous(); }

    /// @brief Coordinate transformation, in-place, on the provided set of points to shift them to the provided reference frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param extr Other reference frame.
    void toThisFromOther(point_list& p_list, const extrinsic& extr) const
        { p_list = (this->extr.inverse() * extr) * p_list.colwise().homogeneous(); }

    /// @brief Coordinate transformation on the provided point to shift it to the provided reference frame.
    /// @param p Point in this entity's frame.
    /// @param extr Other reference frame.
    point toThisFromOther(const point& p, const extrinsic& extr) const
        { return (this->extr.inverse() * extr) * p.homogeneous(); }

    /// @brief Coordinate transformation on the provided set of points to shift them to the provided reference frame.
    /// @param p_list List (a 3xN matrix) of points in this entity's frame.
    /// @param extr Other reference frame.
    point_list toThisFromOther(const point_list& p_list, const extrinsic& extr) const
        { return (this->extr.inverse() * extr) * p_list.colwise().homogeneous(); }
};


} // ForgeScan

#endif // FORGESCAN_ENTITY_H
