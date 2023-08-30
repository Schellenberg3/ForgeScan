#ifndef FORGE_SCAN_SIMULATION_PRIMITIVE_HPP
#define FORGE_SCAN_SIMULATION_PRIMITIVE_HPP

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>
#include <highfive/H5Easy.hpp>

#include "ForgeScan/Common/AABB.hpp"
#include "ForgeScan/Common/Types.hpp"
#include "ForgeScan/Common/Entity.hpp"

// Helper constant for saving data in HDF5 files.
// Used by derived classes and Scene.
// Undefined at the end of Scene.hpp.
#define FS_HDF5_PRIMITIVE_TYPE_NAME_ATTR "type_name"


namespace forge_scan {
namespace simulation {


/// @brief Base class for Primitive geometry types.
struct Primitive : Entity
{
public:
    virtual ~Primitive() { }


    // ***************************************************************************************** //
    // *                              PUBLIC PURE VIRTUAL METHODS                              * //
    // ***************************************************************************************** //


    /// @return String name for this type of Primitive.
    virtual const std::string& getTypeName() const = 0;


    /// @brief Determines if, and where, the line between the start and end points first intersects
    ///        the geometry.
    /// @param start  Start point, relative to the derived class's reference frame.
    /// @param end    End point, relative to the derived class's reference frame.
    /// @param t[out] Parameterization value for the intersection. Values `0<=t<=1` are valid
    ///               on the line segment where `t=0` is the start point and `t=1` is the end point.
    /// @return True if the line intersects and does so in a valid region of the line.
    /// @note If the line DOES NOT intersect we return false with t unchanged.
    virtual bool hit(const Point& start, const Point& end, float& t) const = 0;


    /// @brief Calculates is the point is inside the Primitive.
    /// @param input Point in space.
    /// @param extr  Transformation from the world frame to the frame the `input` is in.
    /// @return True if the point is inside, false if not.
    virtual bool isInside(const Point& input, const Extrinsic& extr) const = 0;


    /// @brief Calculates is the point is inside the Primitive.
    /// @param input Point in space.
    /// @param extr  Transformation from the world frame to the frame the `input` is in.
    /// @param input_this_f[out] The point transformed to the Primitive's frame. Useful if
    ///                          calling `getSignedDistance` or `getNearestSurfacePoint` after this.
    /// @return True if the point is inside, false if not.
    virtual bool isInside(const Point& input, const Extrinsic& extr, Point& input_this_f) const = 0;


    /// @brief Calculates the shortest, signed distance from the point and the Primitive's surface.
    /// @param input Point in space.
    /// @param extr  Transformation from the world frame to the frame the `input` is in.
    /// @return The shortest distance between the point and the surface with negative distances
    ///         being inside the Primitive geometry.
    virtual float getSignedDistance(const Point& input, const Extrinsic& extr) const = 0;


    /// @brief Calculates the shortest, signed distance from the point and the Primitive's surface.
    /// @param input Point in space, relative to the derived class's reference frame.
    /// @return The shortest distance between the point and the surface with negative distances
    ///         being inside the Primitive geometry.
    virtual float getSignedDistance(const Point& input) const = 0;


    /// @brief Calculates the location on the Primitive surface closest to the input.
    /// @param input Point in space.
    /// @param extr  Transformation from the world frame to the frame the `input` is in.
    /// @return A Point located on the primitive's surface.
    virtual Point getNearestSurfacePoint(const Point& input, const Extrinsic& extr) const = 0;


    /// @brief Calculates the location on the Primitive surface closest to the input.
    /// @param input Point in space, relative to the derived class's reference frame.
    /// @return A Point located on the primitive's surface.
    virtual Point getNearestSurfacePoint(const Point& input) const = 0;


    /// @brief Saves the shapes contents into an HDF5 file format.
    /// @param g_primitive Group to add data to.
    virtual void save(HighFive::Group& g_primitive) const = 0;



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Bounding box limits for the geometric primitive.
    ///        Positions are relative to the derived class's reference frame.
    const Point upperAABBbound, lowerAABBbound;


protected:
    // ***************************************************************************************** //
    // *                               PROTECTED CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Protected constructor for derived classes only.
    /// @param extr Transformation from the world frame to the derived class's reference frame. 
    /// @param upperAABBbound Upper bound position, relative to the derived class's reference frame.
    /// @param lowerAABBbound Lower bound position, relative to the derived class's reference frame.
    Primitive(const Extrinsic& extr, const Point& upperAABBbound, const Point& lowerAABBbound)
        : Entity(extr),
          upperAABBbound(upperAABBbound),
          lowerAABBbound(lowerAABBbound)
    {

    }


    /// @brief Checks if a point is within the axis-aligned bounding box (AABB) for the Primitive.
    /// @param input Point to check.
    /// @return True if the point is between the upper and lower AABB bound (or on one of them). False else.
    bool insideBounds(const Point& input) const
    {
        return (input.array() >= lowerAABBbound.array()).all() && (input.array() <= upperAABBbound.array()).all();
    }



    /// @brief Quick axis-aligned bounding box (AABB) check for bounding box intersection of a
    ////        geometric primitive. This prevents needless intersection checks for some rays.
    /// @param start  Starting point of the ray, relative to the derived class's reference frame.
    /// @param end    Ending point of the ray, relative to the derived class's reference frame.
    /// @param t[out] Scaling factor for the ray to intersect the AABB. Describes when the ray first
    ///               hits a face of the AABB. Values 0 <= t <= 1 are valid on the line segment 
    ///               between start and end.
    /// @return True if the ray has any intersection with the primitive's bounding box.
    /// @warning The output variable, `t`, is valid only when this function returns true.
    ///          Otherwise it does not describe an intersection and should not be trusted.
    bool hitAABB(const Point& start, const Point& end, float& t) const
    {
        float tmax;
        const Ray inverse_ray = (end - start).cwiseInverse();
        return AABB::find_bounded_intersection(lowerAABBbound, upperAABBbound, start, inverse_ray, 0, 1, t, tmax);
    }
};


} // namespace simulation
} // namespace forge_scan


#endif // FORGE_SCAN_SIMULATION_PRIMITIVE_HPP