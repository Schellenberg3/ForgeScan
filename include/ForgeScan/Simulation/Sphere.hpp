#ifndef FORGE_SCAN_SIMULATION_SPHERE_HPP
#define FORGE_SCAN_SIMULATION_SPHERE_HPP

#include <memory>
#include <string>

#include "ForgeScan/Simulation/Primitive.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"

// Helper constant for saving data in HDF5 files.
// Used in this class and by Scene.
// Undefined at the end of Scene.hpp.
#define FS_HDF5_SPHERE_R_ATTR "radius"


namespace forge_scan {
namespace simulation {


/// @brief A simple analytical Sphere.
/// @note The reference frame for the Sphere is located at its center.
struct Sphere : public Primitive
{
    /// @brief Constructs an analytical sphere with the given radius at the specified location.
    /// @param radius Radius value for the sphere. Default is 1 unit.
    /// @param extr  Transformation to the world frame to the center of the Sphere.
    Sphere(const float& radius = 1, const Extrinsic& extr = Extrinsic::Identity())
        : Primitive(extr,
                    getAABBbound(std::abs(radius)), 
                    getAABBbound(-1 * std::abs(radius))),
          radius(std::abs(radius)),
          radius_squared(this->radius * this->radius)
    {

    }


    /// @brief Constructor for a shared pointer to a Sphere.
    /// @param radius Radius value for the sphere. Default is 1 unit.
    /// @param extr  Transformation to the world frame to the center of the Sphere.
    /// @return Shared pointer to a Sphere.
    static std::shared_ptr<Sphere> create(const float& radius = 1,
                                          const Extrinsic& extr = Extrinsic::Identity())
    {
        return std::shared_ptr<Sphere>(new Sphere(radius, extr));
    }


    /// @brief Constructor for a shared pointer to a Sphere.
    /// @param parser Arg Parser with arguments to construct a Sphere from.
    /// @return Shared pointer to a Sphere.
    static std::shared_ptr<Sphere> create(const utilities::ArgParser& parser)
    {
        Extrinsic extr = Extrinsic::Identity();
        extr.translation().x() = parser.getCmdOption<float>("--x", 0);
        extr.translation().y() = parser.getCmdOption<float>("--y", 0);
        extr.translation().z() = parser.getCmdOption<float>("--z", 0);
        float radius = parser.getCmdOption<float>("--radius", 1);
        return std::shared_ptr<Sphere>(new Sphere(radius, extr));
    }



    // ***************************************************************************************** //
    // *                            PUBLIC VIRTUAL METHOD OVERRIDES                            * //
    // ***************************************************************************************** //


    const std::string& getTypeName() const override final
    {
        static const std::string type_name("Sphere");
        return type_name;
    }


    bool hit(const Point& start, const Point& end, float& t) const override final
    {
        float unused_intersection_time;
        if ( !hitAABB(start, end, unused_intersection_time) )
        {
            return false;
        }

        // Adapted from https://stackoverflow.com/questions/6533856 with a partial quadratic solver
        // for only the real-valued solutions of the intersection.
        // Note that we require the start/end point to be relative to the Sphere's reference frame 
        // thus the value of "center" used in the references above is always equal to zero here.
        // See also: http://paulbourke.net/geometry/circlesphere/.

        // Quadratic equation for intersection:
        //     0 = A*(x*x) + B*x + C
        float A = (start - end).array().pow(2).sum();
        float C = start.array().pow(2).sum() - radius_squared;
        float B = end.array().pow(2).sum() - A - C - radius_squared;

        // Find quadratic equation determinant.
        // Early exit if negative; a complex solutions mean no intersection.
        float D = B*B - 4*A*C;
        if (D < 0)
        {
            return false;
        }

        // Pre-calculations help us optimize the quadratic formula. And checking the sign of B lets
        // us utilize an numerically stable form in which only addition OR subtraction is  required.
        //     D = sqrt(B*B - 4*A*C)
        //     if B < 0
        //         X_1 = (-B + D) / 2*A
        //         X_2 = 2*C / (-B + D)
        //         (Leads to adding two positives)
        //     if B >= 0
        //         X_1 = (-B - D) / 2*A
        //         X_2 = 2*C / (-B - D)
        // In short, the first case lets us add two positives and the second lets us subtract two
        // negatives. This is ideal as it avoids any case where we subtract quantities with the
        // same sign. In cases where these values are similar in magnitude (for this case, when 
        // 4*A*C is small) this leads to imprecision in rounding. For details on this numeric 
        // stability see: https://people.csail.mit.edu/bkph/articles/Quadratics.pdf

        // Both cases require the following values which we may pre-compute
        D  = std::sqrt(D);
        A *= 2;
        C *= 2;
        B *= -1;

        if (B > 0)
        {
            B += D;
        }
        else
        {
            B -= D;
        }
        t = C / B;
        float x = B / A;

        if (t >= 0)
        {
            // Find minimum if both are positive. Else, leave t unchanged as the other solution is non-positive.
            if (x > 0) t = std::min(t, x);
        }
        else
        {
            // Find maximum if both are negative. Else, set t to the other solution, which must be non-negative.
            t = x < 0 ? std::max(t, x) : x;
        }
        // For the case t == 0 the answers are the same so no comparison is needed.

        // In this, 0 <= t <= 1 indicates a point between the two values of interest.
        return ( 0 <= t && t <= 1 );
    }


    virtual bool isInside(const Point& input, const Extrinsic& extr) const override final
    {
        return this->getSignedDistance(input, extr) < 0;
    }


    virtual bool isInside(const Point& input, const Extrinsic& extr, Point& input_this_f) const override final
    {
        input_this_f = this->getToThisFromOther(extr) * input.homogeneous();
        return this->getSignedDistance(input_this_f) < 0;
    }


    float getSignedDistance(const Point& input, const Extrinsic& extr) const override final
    {
        return this->getSignedDistance(this->getToThisFromOther(extr) * input.homogeneous());
    }


    float getSignedDistance(const Point& input) const override final
    {
        return input.norm() - radius;
    }


    Point getNearestSurfacePoint(const Point& input, const Extrinsic& extr) const override final
    {
        return this->getNearestSurfacePoint(this->getToThisFromOther(extr) * input.homogeneous());
    }


    Point getNearestSurfacePoint(const Point& input) const override final
    {
        return input.array() * (radius / input.norm());
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Sphere radius in world units.
    const float radius;


private:
    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Constructor helper for generating a sphere's AABB bounds.
    /// @param radius Radius of the sphere.
    ///               Pass as positive for the upper bound. Pass as negative for the lower bound.
    /// @return Axis-aligned bounding box point for the upper or lower, depending on the radius' sign.
    static Point getAABBbound(const float& radius)
    {
        return Point(radius, radius, radius);
    }

    /// @brief Saves the shapes contents into an HDF5 file format.
    /// @param g_primitive Group to add data to.
    void save(HighFive::Group& g_primitive) const override final
    {
        g_primitive.createAttribute(FS_HDF5_PRIMITIVE_TYPE_NAME_ATTR, this->getTypeName());
        g_primitive.createAttribute(FS_HDF5_SPHERE_R_ATTR,            this->radius);
    }


    /// @brief Helper constant for the ray hit calculation.
    const float radius_squared;
};


} // namespace simulation
} // namespace forge_scan


#endif // FORGE_SCAN_SIMULATION_SPHERE_HPP
