#include <iostream>
#include <numeric>

#include "ForgeScan/forgescan_types.h"
#include "ForgeScan/TSDF/grid.h"

namespace ForgeScan {
namespace TSDF {


/// @details This implements the ray traversal methods declared in the Grid class.
///
/// The methods in this file follow the the Amanatides-Woo algorithm for fast voxel traversal. See:
///     http://www.cse.yorku.ca/~amana/research/grid.pdf
///
/// Essentially, this treats the ray as a line parametrized by a variable for time. This time factor describes how
/// long the distance we "walk" from the given start position. We then consider "walking" each cartesian direction
/// and find the next voxel by selecting the direction that we have spent the least time "walking" along.
///
/// For all of these methods and the axis-aligned bounding-box (AABB) check, it is important that the Grid is
/// aligned to the axis and that any provided points are transformed to the relevant coordinate system. Other code
/// should handle this well before these functions are called. But it is important to know if you expand or edit these.


/// Index constants for functions in this file.
static constexpr int X = 0, Y = 1, Z = 2;

/// Constant reference to double infinity.
static constexpr double DOUBLE_INFINITY = std::numeric_limits<double>::infinity();


/// @brief Calculates and returns the normal and length for the ray between the two points.
/// @param rs Start of the ray.
/// @param re End of the ray.
/// @param normal[out] The returned normal for the vector.
/// @param length[out] Length between the start and end point.
static inline void getRayNormalAndLength(const point& rs, const point& re, Vector3d& normal, double& length)
{
    const Vector3d ray = re - rs;
    length = ray.norm();
    normal = ray / length;
}


/// @brief Checks if, and then where, the ray intersects the Axis-Aligned Bounding Box (AABB).
///        Optimized for lower bounds at the origin and pre-computed inverse of the ray's normal.
/// @param bound  Upper bound of the box. Since it is zero-bounded this is equivelent to the box's dimensions.
/// @param origin Origin of the ray to check.
/// @param inverse_direction The element-wise inverse of unit vector direction for the ray.
/// @param ts Start time for the intersection check. A value of 0 begins the check at the ray origin. Negative values are valid.
/// @param te End time for the intersection check. This is in multiples of the unit vector from the origin.
/// @param ts_adj Adjusted start time. When the ray first enters the AABB along any axis. May be less than the provided ts.
/// @param te_adj Adjusted end time. When the ray first exits the AABB along any axis. May be greater than the provided te.
/// @return False if the ray never enters the AABB. True otherwise.
/// @warning This assumes that all values in `bound` are positive, but does not check this.
/// @warning This requires that `direction` is normalized but does neither checks nor performs this operation.
/// @warning This assumes `te` is greater than `ts` but does not check this. Incorrect times may cause the function
///          to incorrectly conclude that a ray does not intersect and end early.
static bool inline zeroBoundedAABBintersection(const Vector3d& bound, const point& origin, const Vector3d& inverse_direction,
                                               const double& ts, const double& te, double& ts_adj, double& te_adj)
{
    double t_min_y, t_max_y, t_min_z, t_max_z;

    /// Minimum distance = ([lower bounds] - [ray origin]) / [direction]
    const Vector3d min_dist =  -1 * origin.array() * inverse_direction.array();

    /// Maximum distance = ([Upper bounds] - [ray origin]) / [direction]
    const Vector3d max_dist = (bound - origin).array() * inverse_direction.array();

    if (inverse_direction[0] >= 0)
    {
        ts_adj = min_dist[0];
        te_adj = max_dist[0];
    } else {
        ts_adj = max_dist[0];
        te_adj = min_dist[0];
    }

    if (inverse_direction[1] >= 0)
    {
        t_min_y = min_dist[1];
        t_max_y = max_dist[1];
    } else {
        t_min_y = max_dist[1];
        t_max_y = min_dist[1];
    }

    if (ts_adj > t_max_y || t_min_y > te_adj) return false;

    if (t_min_y > ts_adj) ts_adj = t_min_y;
    if (t_max_y < te_adj) te_adj = t_max_y;

    if (inverse_direction[2] >= 0)
    {
        t_min_z = min_dist[2];
        t_max_z = max_dist[2];
    } else {
        t_min_z = max_dist[2];
        t_max_z = min_dist[2];
    }

    if (ts_adj > t_max_z || t_min_z > te_adj) return false;

    if (t_min_z > ts_adj) ts_adj = t_min_z;
    if (t_max_z < te_adj) te_adj = t_max_z;

    return (ts_adj < te && ts < te_adj);
}


/// @brief Takes voxel traversal parameters and updates them in case initial assumptions were off.
/// @param step  Direction of travel (increment or decrement) along each axis. Assumes the initial guess in incrementing (+1).
/// @param delta Time step the resolution divided by the normal in each direction. Assumes the magnitude is pre-calculated
///              and, if needed, ensures the value is positive.
/// @param time  Stat time for the ray. Assumes this is the adjusted start time from an AABB check. Updates this with how far
///              the start point is for the first voxel boundary it must hit.
/// @param resolution  The resolution of the grid - edge length of the square voxels.
/// @param start_voxel The voxel the vector starts in.
/// @param start_point Starting position of the vector.
/// @param normal         Normal vector for the ray.
/// @param inverse_normal Pre-computed inverse of the normal vector.
static inline void correctTraversalInfo(int step[3], double delta[3], double time[3], const double& resolution, const index& start_voxel,
                                        const Vector3d& start_point, const Vector3d& normal, const Vector3d& inverse_normal)
{
    /// Explicitly running this for X, Y, and Z indicies is noticeably faster than a for loop.
    /// So we trade compact readability for speed.

    /// The voxel traversal algorithm requires a voxel's origin to be at the lower bound (rather than its center). To calculate the
    /// traversal information we must virtually shift the initial point.
    const point start_point_shift = start_point.array() + 0.5 * resolution;

    if (normal[X] > 0) {
        /// If we increase in a direction, increase the start time with how far it is to the beginning voxel boundary.
        time[X]  += ((start_voxel[X] + 1) * resolution - start_point_shift[X]) * inverse_normal[X];
    } else if (normal[X] != 0) {
        /// If we decrease in a direction, we need to flip our assumptions: decrement, correct the negative sign in delta, and
        /// update the start time with how far it is to the beginning of the voxel we are in.
        step[X]  *= -1;
        delta[X] *= -1;
        time[X]  += ((start_voxel[X] - 0) * resolution - start_point_shift[X]) * inverse_normal[X];
    } else {
        /// In the rare case where the normal is EXACTLY zero then set the time to infinity so we never step in this direction.
        time[X]  = DOUBLE_INFINITY;
    }

    /// Repeat above for Y...
    if (normal[Y] > 0) {
        time[Y]  += ((start_voxel[Y] + 1) * resolution - start_point_shift[Y]) * inverse_normal[Y];
    } else if (normal[Y] != 0) { step[Y]  *= -1;
        delta[Y] *= -1;
        time[Y]  += ((start_voxel[Y] - 0) * resolution - start_point_shift[Y]) * inverse_normal[Y];
    } else { time[Y]  = DOUBLE_INFINITY; }

    /// Repeat above for Z...
    if (normal[Z] > 0) {
        time[Z]  += ((start_voxel[Z] + 1) * resolution - start_point_shift[Z]) * inverse_normal[Z];
    } else if (normal[Z] != 0) {
        step[Z]  *= -1;
        delta[Z] *= -1;
        time[Z]  += ((start_voxel[Z] - 0) * resolution - start_point_shift[Z]) * inverse_normal[Z];
    } else { time[Z]  = DOUBLE_INFINITY; }
}


bool implementAddRayUpdate(Grid& grid, const point& rs, const point& re, const Voxel::Update& update)
{
    /// Start time and end time (also length) for the user's ray.
    double ts = 0, te = 0;

    /// Adjusted start and end time for when the user's ray intersects the grid.
    double ts_adj = 0, te_adj = 0;

    /// Normalized direction for the ray (with pre-computed inverse values for AABB intersection and traversal info).
    Vector3d normal(0, 0, 0), inverse_normal(0, 0, 0);

    getRayNormalAndLength(rs, re, normal, te);
    inverse_normal = normal.cwiseInverse();

    /// Query when the line segment intersects the AABB. We can either exit early or narrow our trace to only valid voxels.
    bool valid_intersection = zeroBoundedAABBintersection(grid.properties.dimensions, rs, inverse_normal,
                                                          ts, te, ts_adj, te_adj);
    if (!valid_intersection) return false;

    /// Update the end times to either the users bounds or the valid intersecting bounds.
    ts_adj = std::max(ts_adj, ts);
    te_adj = std::min(te_adj, te);

    /// Shift the starting point up by the adjusted time.
    const point rs_adj = rs + normal * ts_adj;

    /// Current index within the grid.
    index c_idx = grid.pointToIndex(rs_adj);

    /// Direction of travel (increment or decrement) along the respective axis. Assume we increment.
    int step[3] = {1, 1, 1};

    /// The amount of time to move one voxel length along each axis based on the ray's direction. Assume inverse normal is positive.
    double delta[3] = { grid.properties.resolution*inverse_normal[X],
                        grid.properties.resolution*inverse_normal[Y],
                        grid.properties.resolution*inverse_normal[Z] };

    /// Cumulative time traveled along the respective axis. Assume we have no adjustment from the start time.
    double time[3] = {ts_adj, ts_adj, ts_adj};

    /// Update step and time, and delta in case our assumptions were wrong.
    correctTraversalInfo(step, delta, time, grid.properties.resolution, c_idx, rs_adj, normal, inverse_normal);

    try {
        /// Loop runs until each direction has just passed the adjusted end time.
        /// By checking, and breaking, in the if statements we make sure that when we exit the loop c_idx is the last voxel we added.
        while (true) {
            /// Update the current element. Starting with the initial voxel.
            grid.at(c_idx).update(update);
            if (time[X] < time[Y] && time[X] < time[Z]) {
                time[X]  += delta[X];
                if (time[X] > te_adj && time[Y] > te_adj && time[Z] > te_adj) break;
                c_idx[X] +=  step[X];
            } else if (time[Y] < time[Z]) {
                time[Y]  += delta[Y];
                if (time[X] > te_adj && time[Y] > te_adj && time[Z] > te_adj) break;
                c_idx[Y] +=  step[Y];
            } else {
                time[Z]  += delta[Z];
                if (time[X] > te_adj && time[Y] > te_adj && time[Z] > te_adj) break;
                c_idx[Z] +=  step[Z];
            }
        }
    } catch (const std::out_of_range& e) {
        std::cerr << "[Grid::implementAddRayUpdate] " << e.what() << "\n";
    }
    return true;
}


bool implementAddRayTSDF(Grid& grid, const point &origin, const point &sensed, Metrics::Ray& ray_metrics)
{
    /// Far time for the TSDF ray, also the length from sensed to origin.
    double tf = 0;

    /// Adjusted positive, negative, and far time for when the user's ray intersects the grid.
    double tp_adj = 0, tn_adj = 0, tf_adj = 0;

    /// Normalized direction for the ray (with pre-computed inverse values for AABB intersection and traversal info).
    Vector3d normal(0, 0, 0), inverse_normal(0, 0, 0);

    /// Get the ray from sensed to origin; the order is important for the signs of the normal later on.
    getRayNormalAndLength(sensed, origin, normal, tf);
    inverse_normal = normal.cwiseInverse();

    /// Query when the line segment intersects the AABB. We can either exit early or narrow our trace to only valid voxels.
    bool valid_intersection = zeroBoundedAABBintersection(grid.properties.dimensions, sensed, inverse_normal,
                                                          grid.properties.min_dist, tf, tn_adj, tf_adj);
    if (!valid_intersection) return false;

    /// Update the end times to either the users bounds or the valid intersecting bounds.
    tf_adj = std::min(tf_adj, tf);
    tp_adj = std::min(tf_adj, grid.properties.max_dist);
    tn_adj = std::max(tn_adj, grid.properties.min_dist);

    /// Shift the sensed point back by the adjusted negative time.
    const point sensed_adj = sensed + normal * tn_adj;

    /// Current index within the grid.
    index c_idx = grid.pointToIndex(sensed_adj);

    /// Direction of travel (increment or decrement) along the respective axis. Assume we increment.
    int step[3] = {1, 1, 1};

    /// The amount of time to move one voxel length along each axis based on the ray's direction. Assume inverse normal is positive.
    double delta[3] = { grid.properties.resolution*inverse_normal[X],
                        grid.properties.resolution*inverse_normal[Y],
                        grid.properties.resolution*inverse_normal[Z] };

    /// Cumulative time traveled along the respective axis. Assume we have no adjustment from the negative time.
    double time[3] = {tn_adj, tn_adj, tn_adj};

    /// Update step and time, and delta in case our assumptions were wrong.
    correctTraversalInfo(step, delta, time, grid.properties.resolution, c_idx, sensed_adj, normal, inverse_normal);

    // Perform updates withing the truncation distance. Moving from neg_dist to pos_dist.
    Voxel::Update update(static_cast<float>(tn_adj));

    Voxel *voxel_ref;

    try {
        /// First walk is from the adjusted negative location to the adjusted positive location.
        while (time[X] <= tp_adj || time[Y] <= tp_adj || time[Z] <= tp_adj) {
            voxel_ref = &grid.at(c_idx);
            if (voxel_ref->views == 0) ++ray_metrics.first;
            voxel_ref->update(update);
            if (voxel_ref->var > ray_metrics.max_variance_update) ray_metrics.max_variance_update = voxel_ref->var;

            if (time[X] < time[Y] && time[X] < time[Z]) {
                time[X]  += delta[X];
                c_idx[X] +=  step[X];
                update.dist = static_cast<float>(time[X]);
            } else if (time[Y] < time[Z]) {
                time[Y]  += delta[Y];
                c_idx[Y] +=  step[Y];
                update.dist = static_cast<float>(time[Y]);
            } else {
                time[Z]  += delta[Z];
                c_idx[Z] +=  step[Z];
                update.dist = static_cast<float>(time[Z]);
            }
            ++ray_metrics.updates;
            ++ray_metrics.views;
        }

        /// Second walk is from the adjusted positive location to the adjusted far location.
        while (time[X] <= tf_adj || time[Y] <= tf_adj || time[Z] <= tf_adj) {
            voxel_ref = &grid.at(c_idx);
            if (voxel_ref->views == 0) ++ray_metrics.first;
            voxel_ref->setViewUpdateFlag();

            if (time[X] < time[Y] && time[X] < time[Z]) {
                time[X]  += delta[X];
                c_idx[X] +=  step[X];
            } else if (time[Y] < time[Z]) {
                time[Y]  += delta[Y];
                c_idx[Y] +=  step[Y];
            } else {
                time[Z]  += delta[Z];
                c_idx[Z] +=  step[Z];
            }
            ++ray_metrics.views;
        }
    } catch (const std::out_of_range& e) {
        std::cerr << "[Grid::implementAddRayTSDF] " << e.what() << "\n";
    }
    return true;
}


} // TSDF
} // ForgeScan
