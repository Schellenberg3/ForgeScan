#ifndef FORGE_SCAN_COMMON_ENTITY_HPP
#define FORGE_SCAN_COMMON_ENTITY_HPP


#include "ForgeScan/Common/Types.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"


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

    /// @brief Sets the Extrinsic's translation based on the parsed arguments.
    /// @param parser Arguments for the translation.
    /// @param extr Extrinsic to be set.
    static void setTranslation(const utilities::ArgParser& parser, Extrinsic& extr)
    {
        extr.translation().x() = parser.getCmdOption<float>(Entity::parse_x, Entity::default_translation);
        extr.translation().y() = parser.getCmdOption<float>(Entity::parse_y, Entity::default_translation);
        extr.translation().z() = parser.getCmdOption<float>(Entity::parse_z, Entity::default_translation);
    }



    /// @brief Sets the Extrinsic's rotation based on the parsed arguments.
    /// @param parser Arguments for the translation.
    /// @param extr Extrinsic to be set.
    static void setRotation(const utilities::ArgParser& parser, Extrinsic& extr)
    {
        // Its silly to store this, but I couldn't quickly find an intuitive way to reset
        // only the rotation part of the Extrinsic.
        Eigen::Vector3f translation = extr.translation();
        extr.setIdentity();

        float scale = parser.cmdOptionExists(Entity::parse_rotation_degrees) ? M_PI / 180.0f : 1;      
        extr.rotate(Eigen::AngleAxisf(scale * parser.getCmdOption<float>(Entity::parse_rx, Entity::default_rotation), Ray::UnitX()) *
                    Eigen::AngleAxisf(scale * parser.getCmdOption<float>(Entity::parse_ry, Entity::default_rotation), Ray::UnitY()) *
                    Eigen::AngleAxisf(scale * parser.getCmdOption<float>(Entity::parse_rz, Entity::default_rotation), Ray::UnitZ()));
        extr.translation() = translation;
    }


    static const float default_translation;

    static const std::string parse_x, parse_y, parse_z;

    static const float default_rotation;

    static const std::string parse_rx, parse_ry, parse_rz;

protected:
    /// @brief Extrinsic transformation from the world coordinates to the entity.
    Extrinsic extr;
};


/// @brief Default translation (in each direction axis).
const float Entity::default_translation = 0.0;

/// @brief ArgParser key for the translation in X, Y and Z.
const std::string Entity::parse_x  = std::string("--x"),
                  Entity::parse_y  = std::string("--y"),
                  Entity::parse_z  = std::string("--z");

/// @brief Default rotation (about each axis).
const float Entity::default_rotation = 0.0;

/// @brief ArgParser key for the rotation around X, Y and Z.
const std::string Entity::parse_rx = std::string("--rx"),
                  Entity::parse_ry = std::string("--ry"),
                  Entity::parse_rz = std::string("--rz");

} // namespace forge_scan


#endif // FORGE_SCAN_COMMON_ENTITY_HPP
