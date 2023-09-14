#ifndef FORGE_SCAN_SIMULATION_BOX_HPP
#define FORGE_SCAN_SIMULATION_BOX_HPP

#include <memory>

#include "ForgeScan/Simulation/Primitive.hpp"
#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Math.hpp"

// Helper constant for saving data in HDF5 files.
// Used in this class and by Scene.
// Undefined at the end of Scene.hpp.
#define FS_HDF5_BOX_L_ATTR "length"
#define FS_HDF5_BOX_W_ATTR "width"
#define FS_HDF5_BOX_H_ATTR "hight"



namespace forge_scan {
namespace simulation {


/// @brief A simple analytical box object.
/// @note The reference frame for a Box is located at its center and oriented so the length is
///       aligned with the X-axis, width is aligned with the Y-axis, and width is aligned with
///       the Z-axis.
struct Box : public Primitive
{
public:
    /// @brief Constructs an analytical box of the provided dimensions.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param extr  Transformation to the world frame to the center of the Box.
    Box(const float& l, const float& w, const float& h, const Extrinsic& extr = Extrinsic::Identity())
        : Primitive(extr, getAABBbound(l, w, h, true), getAABBbound(l, w, h, false)),
          length(l),
          width(w),
          height(h)
    {

    }


    /// @brief Constructor for a shared pointer to a Box.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param extr  Transformation to the world frame to the center of the Box.
    /// @return Shared pointer to a Box.
    static std::shared_ptr<Box> create(const float& l, const float& w, const float& h,
                                       const Extrinsic& extr = Extrinsic::Identity())
    {
        return std::shared_ptr<Box>(new Box(l, w, h, extr));
    }


    /// @brief Constructor for a shared pointer to a Box.
    /// @param parser ArgParser with arguments to construct a Box from.
    /// @return Shared pointer to a Box.
    static std::shared_ptr<Box> create(const utilities::ArgParser& parser)
    {
        Extrinsic extr = Extrinsic::Identity();
        Entity::setRotation(parser, extr);
        Entity::setTranslation(parser, extr);

        return std::shared_ptr<Box>(new Box(parser.get<float>(Box::parse_l, Box::default_length),
                                            parser.get<float>(Box::parse_w, Box::default_width),
                                            parser.get<float>(Box::parse_h, Box::default_height), extr));
    }


    /// @return Help message for constructing a Box with ArgParser.
    static std::string helpMessage()
    {
        return "A Box may be added with the following arguments:"
               "\n\t" + Box::help_string +
               "\nIf the optional arguments are not provided, the default values are:"
               "\n\t" + Box::default_arguments;
    }


    // ***************************************************************************************** //
    // *                            PUBLIC VIRTUAL METHOD OVERRIDES                            * //
    // ***************************************************************************************** //


    const std::string& getTypeName() const override final
    {
        return Box::type_name;
    }


    bool hit(const Point& start, const Point& end, float& t) const override final
    {
        return this->hitAABB(start, end, t);
    }


    bool isInside(const Point& input, const Extrinsic& extr) const override final
    {
        Point input_this_f = this->getToThisFromOther(extr) * input.homogeneous();
        return this->insideBounds(input_this_f);
    }


    bool isInside(const Point& input, const Extrinsic& extr, Point& input_this_f) const override final
    {
        input_this_f = this->getToThisFromOther(extr) * input.homogeneous();
        return this->insideBounds(input_this_f);
    }


    float getSignedDistance(const Point& input, const Extrinsic& extr) const override final
    {
        return this->getSignedDistance(this->getToThisFromOther(extr) * input.homogeneous());
    }


    float getSignedDistance(const Point& input) const override final
    {
        if (this->insideBounds(input))
        {
            return this->getSignedDistanceInside(input);
        }
        return this->getSignedDistanceOutside(input);
    }


    Point getNearestSurfacePoint(const Point& input, const Extrinsic& extr) const override final
    {
        return this->getNearestSurfacePoint(this->getToThisFromOther(extr) * input.homogeneous());
    }


    Point getNearestSurfacePoint(const Point& input) const override final
    {
        if (this->insideBounds(input))
        {
            return this->getNearestSurfacePointInside(input);
        }
        return this->getNearestSurfacePointOutside(input);
    }



    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS MEMBERS                                  * //
    // ***************************************************************************************** //


    /// @brief Dimensions of the box: length in X-direction, width in Y-direction, and height it Z-direction.
    const float length, width, height;

    static const float default_length, default_width, default_height;

    static const std::string parse_l, parse_w, parse_h;

    static const std::string help_string, default_arguments;

    static const std::string type_name;

private:
    // ***************************************************************************************** //
    // *                                 PRIVATE CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Constructor helper for generating a box's AABB bounds.
    /// @param l The box's total dimension in the X-direction.
    /// @param w The box's total dimension in the Y-direction.
    /// @param h The box's total dimension in the Z-direction.
    /// @param upper If true gets upper bound. If false gets lower bound
    /// @return Axis-aligned bounding box point for the upper or lower, depending on the sign of the dimension.
    static Point getAABBbound(const float& l, const float& w, const float& h, const bool& upper)
    {
        Point bound(l, w, h);
        bound.cwiseAbs();
        if (upper)
        {
            bound.array() *= 0.5;
        }
        else
        {
            bound.array() *= -0.5;
        }
        return bound;
    }


    /// @brief Saves the shapes contents into an HDF5 file format.
    /// @param g_primitive Group to add data to.
    void save(HighFive::Group& g_primitive) const override final
    {
        g_primitive.createAttribute(FS_HDF5_PRIMITIVE_TYPE_NAME_ATTR, this->getTypeName());
        g_primitive.createAttribute(FS_HDF5_BOX_L_ATTR, this->length);
        g_primitive.createAttribute(FS_HDF5_BOX_W_ATTR, this->width);
        g_primitive.createAttribute(FS_HDF5_BOX_H_ATTR, this->height);
    }


    /// @brief Prints information about the Box to the output stream.
    /// @param out Output stream to write to.
    void print(std::ostream& out) const override final
    {
        out << this->getTypeName() << " ";
        Primitive::print(out);
        out << " with dimensions (" << this->length << ", " << this->width << ", " << this->height << ")";
    }


    // ***************************************************************************************** //
    // *                             PRIVATE CLASS HELPER METHODS                              * //
    // ***************************************************************************************** //

    /// @brief Calculates the signed distance for the case where a point is outside of the Box.
    /// @param input_this_f Point, relative to the Box's reference frame.
    /// @return Shortest distance from the box to that point. Always non-negative.
    Point getNearestSurfacePointOutside(const Point& input_this_f) const
    {
        // For each direction if we are between the bounds then we take zero for the delta in that direction.
        // Otherwise we take the maximum value. If we are less than both bounds then sign(`lower` - `input`) is positive
        // while sign(`input` -`upper`) is negative, leading the call to max to store the proper sign and magnitude.
        // The opposite logic lets us store the correct sign and magnitude when we are above both bounds.
        float delta_x = std::max(std::max(lowerAABBbound.x() - input_this_f.x(), 0.0f), input_this_f.x() - upperAABBbound.x());
        float delta_y = std::max(std::max(lowerAABBbound.y() - input_this_f.y(), 0.0f), input_this_f.y() - upperAABBbound.y());
        float delta_z = std::max(std::max(lowerAABBbound.z() - input_this_f.z(), 0.0f), input_this_f.z() - upperAABBbound.z());
        return input_this_f + Point(delta_x, delta_y, delta_z);
    }


    /// @brief Calculates the signed distance for the case where a point is outside of the Box.
    /// @param input_this_f Point, relative to the Box's reference frame.
    /// @return Shortest distance from the box to that point. Always non-negative.
    float getSignedDistanceOutside(const Point& input_this_f) const
    {
        // Uses the same logic as getNearestSurfacePointOutside but skips creating a Point object.
        float delta_x = std::max(std::max(this->lowerAABBbound.x() - input_this_f.x(), 0.0f), input_this_f.x() - this->upperAABBbound.x());
        float delta_y = std::max(std::max(this->lowerAABBbound.y() - input_this_f.y(), 0.0f), input_this_f.y() - this->upperAABBbound.y());
        float delta_z = std::max(std::max(this->lowerAABBbound.z() - input_this_f.z(), 0.0f), input_this_f.z() - this->upperAABBbound.z());
        return std::sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
    }


    /// @brief Calculates the signed distance for the case where a point is inside of the Box.
    /// @param input_this_f Point, relative to the Box's reference frame.
    /// @return Shortest distance from the box to that point. Always non-positive.
    Point getNearestSurfacePointInside(const Point& input_this_f) const
    {
        using namespace utilities::math;

        // For each direction, calculate the signed distance from the internal point to both bounds and store the distance of lesser magnitude.
        float face_x = smallest_magnitude(this->lowerAABBbound.x() - input_this_f.x(), this->upperAABBbound.x() - input_this_f.x());
        float face_y = smallest_magnitude(this->lowerAABBbound.y() - input_this_f.y(), this->upperAABBbound.y() - input_this_f.y());
        float face_z = smallest_magnitude(this->lowerAABBbound.z() - input_this_f.z(), this->upperAABBbound.z() - input_this_f.z());

        // Now, compare the directions to find which has the least magnitude overall.
        if (is_lesser_in_magnitude(face_x, face_y), is_lesser_in_magnitude(face_x, face_y))
        {
            return input_this_f + Point(face_x, 0, 0);

        }
        else if (is_lesser_in_magnitude(face_y, face_z))
        {
            return input_this_f + Point(0, face_y, 0);
        }
        else
        {
            return input_this_f + Point(0, 0, face_z);
        }
    }


    /// @brief Calculates the signed distance for the case where a point is inside of the Box.
    /// @param input_this_f Point, relative to the Box's reference frame.
    /// @return Shortest distance from the box to that point. Always non-positive.
    float getSignedDistanceInside(const Point& input_this_f) const
    {
        // Uses similar logic to getNearestSurfacePointInside but skips creating a Point or finding the direction
        // of the closest face - only finds the magnitude.
        float face_x = std::max(this->lowerAABBbound.x() - input_this_f.x(), input_this_f.x() - this->upperAABBbound.x());
        float face_y = std::max(this->lowerAABBbound.y() - input_this_f.y(), input_this_f.y() - this->upperAABBbound.y());
        float face_z = std::max(this->lowerAABBbound.z() - input_this_f.z(), input_this_f.z() - this->upperAABBbound.z());
        return std::max(std::max(face_x, face_y), face_z);
    }
};


/// @brief String for the class name.
const std::string Box::type_name = "Box";

/// @brief Default dimensions of the box.
const float Box::default_length = 1.0, Box::default_width = 1.0, Box::default_height = 1.0;

/// @brief ArgParser key for the Box length, width, and height.
const std::string Box::parse_l = std::string("--l"),
                  Box::parse_w = std::string("--w"),
                  Box::parse_h = std::string("--h");

/// @brief String explaining what arguments this class accepts.
const std::string Box::help_string =
    Box::translation_help_string + " " + Box::rotation_help_string +
    " [" + Box::parse_l + " <X dimension>]" +
    " [" + Box::parse_w + " <Y dimension>]" +
    " [" + Box::parse_h + " <Z dimension>]";

/// @brief String explaining what this class's default parsed values are.
const std::string Box::default_arguments =
    Box::translation_default_arguments + " " + Box::rotation_default_arguments +
    " " + Box::parse_l + " " + std::to_string(Box::default_length) +
    " " + Box::parse_w + " " + std::to_string(Box::default_width) +
    " " + Box::parse_h + " " + std::to_string(Box::default_height);


} // namespace primitives
} // namespace forge_scan


#endif // FORGE_SCAN_SIMULATION_BOX_HPP
