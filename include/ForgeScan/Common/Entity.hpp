#ifndef FORGE_SCAN_COMMON_ENTITY_HPP
#define FORGE_SCAN_COMMON_ENTITY_HPP


#include "ForgeScan/Common/Types.hpp"


namespace forge_scan {


/// @brief Generic entity (sensor, primitive etc) that exists in 3D space.
/// @note  This essentially wraps the Eigen::Transform class. It provides useful methods for other
///        classes to clarify the semantics of coordinate transformations between one another.
struct Entity
{
    /// @brief Constructs the generic entity at the world origin.
    Entity()
        : extr(Extrinsic::Identity())
    {

    }


    /// @brief Constructs the generic entity.
    /// @param extr Initial pose for the entity.
    explicit Entity(const Extrinsic& extr)
        : extr(extr)
    {

    }

    /********* CONTROL AND VIEW STATE *********/

    /// @brief Returns a read-only reference of the extrinsic matrix.
    const Extrinsic& getExtr() const
    {
        return this->extr;
    }


    /// @brief Sets the extrinsic matrix to provided value.
    /// @param extr New extrinsic pose.
    void setExtr(const Extrinsic& extr)
    {
        this->extr = extr;
    }



    /********* APPLY TRANSFORMATIONS *********/

    /// @brief Performs a body-frame transformation to the entity.
    /// @param other Transformation to apply.
    /// @returns True if the transformation was applied. False if this entity's pose is frozen.
    void transformBodyFrame(const Extrinsic& other)
    {
        this->extr = this->extr * other;
    }


    /// @brief Performs a world-frame transformation to the entity.
    /// @param other Transformation to apply.
    /// @returns True if the transformation was applied. False if this entity's pose is frozen.
    void transformWorldFrame(const Extrinsic& other)
    {
        this->extr = other * this->extr;
    }


    /// @brief Applies a translation to the entity.
    /// @param other Translation to apply.
    void translate(const Translation& other)
    {
        this->extr.translate(other);
    }


    /// @brief Applies a body-frame rotation to the entity.
    /// @param other Rotation matrix to apply.
    void rotateBodyFrame(const Rotation& other)
    {
        this->extr.rotate(other);
    }


    /// @brief Applies a world-frame rotation to the entity.
    /// @param other Rotation matrix to apply.
    void rotateWorldFrame(const Rotation& other)
    {
        this->extr.prerotate(other);
    }


    /********* CALCULATE TRANSFORMATIONS *********/
    /// TODO: write these docstrings

    Extrinsic getToThisFromWorld() const
    {
        return this->extr;
    }


    /// @brief Gets the transformation from this entity to the world frame.
    Extrinsic getToWorldFromThis() const
    {
        return this->extr.inverse();
    }



    Extrinsic getToThisFromOther(const Extrinsic& other) const
    {
        return this->extr.inverse() * other;
    }


    Extrinsic getToOtherFromThis(const Extrinsic& other) const
    {
        return other.inverse() * this->extr;
    }


    Extrinsic getToThisFromOther(const Entity& other) const
    {
        return this->getToThisFromOther(other.getExtr());
    }


    Extrinsic getToOtherFromThis(const Entity& other) const
    {
        return this->getToOtherFromThis(other.getExtr());
    }


    /********* TRANSFORMATIONS FOR EIGEN VECTORS *********/

    /// @brief Transforms the Point into this entity's reference frame, from the world frame.
    /// @param x Input Point.
    /// @returns Transformed Point.
    Point toThisFromWorld(const Point& x) const
    {
        return this->getToThisFromWorld() * x.homogeneous();
    }


    /// @brief Transforms the Point into this entity's reference frame, from the world frame.
    /// @param x Input Point. Transformed in place.
    void toThisFromWorld(Point& x) const
    {
        x = this->getToThisFromWorld() * x.homogeneous();
    }


    /// @brief Transforms the Point into the world frame, from this entity's frame.
    /// @param x Input Point.
    /// @returns Transformed Point.
    Point toWorldFromThis(const Point& x) const
    {
        return this->getToWorldFromThis() * x.homogeneous();
    }


    /// @brief Transforms the Point into the world frame, from this entity's frame.
    /// @param x Input Point. Transformed in place.
    void toWorldFromThis(Point& x) const
    {
        x = this->getToWorldFromThis() * x.homogeneous();
    }


    /// @brief Transforms the Point into this entity's frame, from the other entity's frame.
    /// @param x Input Point.
    /// @returns Transformed Point.
    Point toThisFromOther(const Point& x, const Extrinsic& other) const
    {
        return this->getToThisFromOther(other) * x.homogeneous();
    }


    /// @brief Transforms the Point into this entity's frame, from the other entity's frame.
    /// @param x Input Point. Transformed in place.
    void toThisFromOther(Point& x, const Extrinsic& other) const
    {
        x = this->getToThisFromOther(other) * x.homogeneous();
    }


    /// @brief Transforms the Point into the other entity's frame, from this entity's frame.
    /// @param x Input Point.
    /// @returns Transformed Point.
    Point toOtherFromThis(const Point& x, const Extrinsic& other) const
    {
        return this->getToOtherFromThis(other) * x.homogeneous();
    }


    /// @brief Transforms the Point into the other entity's frame, from this entity's frame.
    /// @param x Input Point. Transformed in place.
    void toOtherFromThis(Point& x, const Extrinsic& other) const
    {
        x = this->getToOtherFromThis(other) * x.homogeneous();
    }


    /********* TRANSFORMATIONS FOR EIGEN MATRICES *********/

    /// @brief Transforms the Point into this entity's reference frame, from the world frame.
    /// @param x Input Points. Transformed in place.
    void toThisFromWorld(PointMatrix& x) const
    {
        x = this->getToThisFromWorld() * x.colwise().homogeneous();
    }


    /// @brief Transforms the Point into the world frame, from this entity's frame.
    /// @param x Input Point. Transformed in place.
    void toWorldFromThis(PointMatrix& x) const
    {
        x = this->getToWorldFromThis() * x.colwise().homogeneous();
    }


    /// @brief Transforms the Point into this entity's frame, from the other entity's frame.
    /// @param x Input Point. Transformed in place.
    void toThisFromOther(PointMatrix& x, const Extrinsic& other) const
    {
        x = this->getToThisFromOther(other) * x.colwise().homogeneous();
    }


    /// @brief Transforms the Point into the other entity's frame, from this entity's frame.
    /// @param x Input Point. Transformed in place.
    void toOtherFromThis(PointMatrix& x, const Extrinsic& other) const
    {
        x = this->getToOtherFromThis(other) * x.colwise().homogeneous();
    }


protected:
    /// @brief Extrinsic transformation from the world coordinates to the entity.
    Extrinsic extr;
};


} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_ENTITY_HPP
