#include <ForgeScan/voxel_grid.h>


namespace ForgeScan {


/// @details This implements the ray traversal methods declared in the VoxelGrid class.
///
/// The methods in this file follow the the Amanatides-Woo algorithm for fast voxel traversal. See:
///     http://www.cse.yorku.ca/~amana/research/grid.pdf
///
/// Essentially, this treats the ray as a line parametrized by a variable for time. This time factor describes how
/// long the distance we "walk" from the given start position. We then consider "walking" each cartesian direction
/// and find the next voxel by selecting the direction that we have spent the least time "walking" along.
///
/// For all of these methods and the axis-aligned bounding-box (AABB) check, it is important that the VoxelGrid is
/// aligned to the axis and that any provided points are transformed to the relevant coordinate system. Other code
/// should handle this well before these functions are called. But it is important to know if you expand or edit these.


/// @TODO: This method works quite well and is very fast. However it tends to miss the final few voxels. At lease how I've
///        implemented it and from my empirical observations. More testing is required here to check the start/end conditions
///        and the initial distances for each voxel.


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
/// @todo This function works but needs to be tested for robustness then optimized for speed. Testing should
///       qualify how results are effected in cases where the assumptions stated in the warnings are incorrect.
static bool zeroBoundedAABBintersection(const Vector3d& bound, const point& origin, const Vector3d& inverse_direction,
                                        const double& ts, const double& te, double& ts_adj, double& te_adj)
{
    double t_min_y, t_max_y, t_min_z, t_max_z;

    // inverse_direction = direction.cwiseInverse();

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


bool VoxelGrid::implementAddRayExact(const VoxelUpdate& update, const point& rs,  const point& re)
{
    Vector3d ray = re - rs;
    double   te = ray.norm();  // also the ray's length
    Vector3d normal = ray / te;
    Vector3d inverse_normal = normal.cwiseInverse();

    double ts = 0;

    // We declare adjusted start and end times (in multiples of the unit vector) so the
    // findRayAlignedBoxIntersection method can adjust our times so we only look at valid voxels.
    double ts_adj, te_adj;

    // Early exit for segments outside of the grid or for equal start/end points. 
    bool valid_segment = zeroBoundedAABBintersection(properties.dimensions, rs, inverse_normal,
                                                     ts, te, ts_adj, te_adj);
    if (valid_segment == false)
        return false;

    // From the adjusted times and given unit-vector multiple times we want to find the max start time
    // and the min end time. This is 
    ts_adj = std::max(ts_adj, ts);
    te_adj = std::min(te_adj, te);

    const point rs_adj = rs + normal * ts_adj;

    Vector3ui current_gidx(0,0,0);

    // For debugging it is helpful to know the end index but it is not used in the traversal algorithm.
    // Vector3ui end_gidx(0,0,0);  
    
    /// [voxel] For each direction, as the ray is traversed, we either increment (+1) or decrement (-1) the index
    /// when we choose to step in that direction.
    int s_x = 1, s_y = 1, s_z = 1;

    /// [time] For each direction, we record the cumulative time spent traversing that direction. We always step in
    /// the direction that has been traversed the least.
    double t_x = ts_adj, t_y = ts_adj, t_z = ts_adj;

    /// [time] For each direction, we find amount of time required to move the distance of one voxel edge length in
    /// that direction. For each movement in a direction, this is added to the cumulative total for that direction.
    double dt_x = 0,  dt_y = 0,  dt_z = 0;

    /// @brief Helper lambda to initialize the step, cumulative time, and delta time for each direction.
    /// @param s_d  Step in the direction. Will be one of increment (+1), decrement (-1), or nothing (0).
    /// @param t_d  Cumulative time moved in that direction.
    ///             This is initialized to the time needed to intersect the next boundary in the given direction.
    /// @param dt_d Time required to move one voxel unit in a direction. 
    /// @param d    Direction. This index retrieves required information from vectors related to the grid and the ray.
    ///             Must be 0 for X, 1 for Y or 2 for Z. But this is not verified.
    auto initTraversalInfo = [&](int& s_d, double& t_d, double& dt_d, const int& d)
    {
        /// TODO: Should this -1 be here? Does that make sense. Ran into boundary condition issues when the
        ///       Far distance point was EXACTLY on the boundary of the grid.
        /// TODO: Can this be optimized or pre-computed?
        /// Calculate the current index for this direction.
        current_gidx[d] = std::max(0, (int)std::floor((rs_adj[d] / properties.resolution) - 1));
    
        dt_d = properties.resolution * inverse_normal[d];
        if (normal[d] > 0)
        {   /// If we increase in a direction, set the start time based on the current index.
            t_d  += abs( (current_gidx[d] * properties.resolution - rs_adj[d]) * inverse_normal[d] );
        }
        else if (normal[d] != 0)
        {   /// If we decrease in a direction...
            s_d  *= -1;  /// Ensures that we decrement the current index in this direction with each step in this direction.
            dt_d *= -1;  /// Ensures that we increase the traversal time with each step, correcting the negative sign from above.
            /// Use the previous index in the direction when setting its start time.
            t_d  += abs( ((current_gidx[d] - 1) * properties.resolution - rs_adj[d]) * inverse_normal[d] );
        }
        else
        {   /// In the rare case where the normal is EXACTLY zero we set the step to zero...
            s_d  = 0;          /// Set the step size to zero.
            dt_d = te_adj;  /// Set the travel time update to the final time.
            t_d  = te_adj;  /// Set the initial time to the final time.
        }
    };

    initTraversalInfo(s_x, t_x, dt_x, 0);
    initTraversalInfo(s_y, t_y, dt_y, 1);
    initTraversalInfo(s_z, t_z, dt_z, 2);

    /// NOTE: The exact start/end voxels hit tend to be a bit off from what might be "exact".
    ///       I believe that there may be slight differences with rounding and from voxel resolution.
    ///       But I will return to this another time.
    at(current_gidx).update(update);  // update current element befor entering the loop.
    while (t_x <= te_adj || t_y <= te_adj || t_z <= te_adj)
    {
        if (t_x < t_y && t_x < t_z)
        {
            current_gidx[0] += s_x;
            t_x += dt_x;
        }
        else if (t_y < t_z) // Y is implicitly less than X 
        {
            current_gidx[1] += s_y;
            t_y += dt_y;
        }
        else // Z is implicitly less than both X and Y 
        {
            current_gidx[2] += s_z;
            t_z += dt_z;
        }
        at(current_gidx).update(update);
    }
    return true;
}


/// @note: This method works quite well and is very fast. However it tends to miss the final 1 to 3 voxels. The exit condition
///        is a bit imperfect. Maybe some boolean flags rather than comparisons will correct this. But this is a minor error at the moment.
bool VoxelGrid::implementAddRayTSDF(const point &origin, const point &sensed, RayRecord& ray_record)
{
    /// [Distance] Adjusted traversal distances that are inside the grid. Given a grid's truncation distance the adjusted values follow:
    ///  NEGATIVE TRUCATION DISTANCE <= t_neg_adj <= t_pos_adj <= POSITIVE TRUCATION DISTANCE,
    ///  t_pos_adj <= t_far_adj
    ///  In some conditions the adjusted negative value may be greater than zero (and the adjusted positive value less than zero). 
    double t_pos_adj = 0, t_neg_adj = 0, t_far_adj = 0;
    Vector3ui current_gidx(0,0,0);
    VoxelUpdate update(0);

    Vector3d ray    = origin - sensed;
    double t_far    = ray.norm();
    Vector3d normal = ray / t_far;
    Vector3d inverse_normal = normal.cwiseInverse();

    // Early exit for segments outside of the grid or for equal start/end points. 
    bool intersects_grid = zeroBoundedAABBintersection(properties.dimensions, sensed, inverse_normal,
                                                       properties.min_dist, t_far,
                                                       t_neg_adj, t_far_adj);

    t_far_adj = std::min(t_far_adj, t_far);
    t_pos_adj = std::min(t_far_adj, properties.max_dist);
    t_neg_adj = std::max(t_neg_adj, properties.min_dist);

    bool correct_traversal_direction = t_neg_adj < t_far_adj;
    if (intersects_grid == false || correct_traversal_direction == false)
        return false;

    const point neg_dist = sensed + normal * t_neg_adj;

    /// [Voxel] For each direction, as the ray is traversed, we either increment (+1) or decrement (-1) the index
    /// when we choose to step in that direction.
    int s_x = 1, s_y = 1, s_z = 1;

    /// [Distance] For each direction, we record the cumulative traveled distance that direction. We always step in
    /// the direction that has been traversed the least.
    double t_x = t_neg_adj, t_y = t_neg_adj, t_z = t_neg_adj;

    /// [Distance] For each direction, we find amount of travel required to move the distance of one voxel edge length in
    /// that direction. For each movement in a direction, this is added to the cumulative total for that direction.
    double dt_x = 0,  dt_y = 0,  dt_z = 0;


    /// @brief Helper lambda to initialize the step, cumulative time, and delta time for each direction.
    /// @param s_d  Step in the direction. Will be one of increment (+1), decrement (-1), or nothing (0).
    /// @param t_d  Cumulative time moved in that direction.
    ///             This is initialized to the time needed to intersect the next boundary in the given direction.
    /// @param dt_d Time required to move one voxel unit in a direction. 
    /// @param d    Direction. This index retrieves required information from vectors related to the grid and the ray.
    ///             Must be 0 for X, 1 for Y or 2 for Z. But this is not verified.
    auto initTraversalInfo = [&](int& s_d, double& t_d, double& dt_d, const int& d)
    {
        /// TODO: Should this -1 be here? Does that make sense. Ran into boundary condition issues when the
        ///       Far distance point was EXACTLY on the boundary of the grid.
        /// TODO: Can this be optimized or pre-computed?
        /// Calculate the current index for this direction.
        current_gidx[d] = std::max(0, (int)std::floor((neg_dist[d] / properties.resolution) - 1));
    
        dt_d = properties.resolution * inverse_normal[d];
        if (normal[d] > 0)
        {   /// If we increase in a direction, set the start time based on the current index.
            t_d  += abs( (current_gidx[d] * properties.resolution - neg_dist[d]) * inverse_normal[d] );
        }
        else if (normal[d] != 0)
        {   /// If we decrease in a direction...
            s_d  *= -1;  /// Ensures that we decrement the current index in this direction with each step in this direction.
            dt_d *= -1;  /// Ensures that we increase the traversal time with each step, correcting the negative sign from above.
            /// Use the previous index in the direction when setting its start time.
            t_d  += abs( ((current_gidx[d] - 1) * properties.resolution - neg_dist[d]) * inverse_normal[d] );
        }
        else
        {   /// In the rare case where the normal is EXACTLY zero we set the step to zero...
            s_d  = 0;          /// Set the step size to zero.
            dt_d = t_far_adj;  /// Set the travel time update to the final time.
            t_d  = t_far_adj;  /// Set the initial time to the final time.
        }
    };

    initTraversalInfo(s_x, t_x, dt_x, 0);
    initTraversalInfo(s_y, t_y, dt_y, 1);
    initTraversalInfo(s_z, t_z, dt_z, 2);

    VoxelElement *element_ref;

    // Perform updates withing the truncation distance. Moving from neg_dist to pos_dist.
    update.dist = t_neg_adj;
    while (t_x <= t_pos_adj || t_y <= t_pos_adj || t_z <= t_pos_adj)
    {
        try {
            element_ref = &at(current_gidx);
            if (element_ref->views == 0) ++ray_record.first;
            element_ref->update(update);
            if (element_ref->var > ray_record.max_variance_update) ray_record.max_variance_update = element_ref->var;
        } catch (const std::out_of_range& e) {
            break; // Break if the next update tried to put us out of range.
                   // Technically, the next loop would not execute after this.
                   // Perhaps we could return rather than perform that check.
        }
        if (t_x < t_y && t_x < t_z)
        {
            update.dist = t_x;
            current_gidx[0] += s_x;
            t_x += dt_x;
        }
        else if (t_y < t_z) // Y is implicitly less than X 
        {
            update.dist = t_y;
            current_gidx[1] += s_y;
            t_y += dt_y;
        }
        else // Z is implicitly less than both X and Y 
        {
            update.dist = t_z;
            current_gidx[2] += s_z;
            t_z += dt_z;
        }
        ++ray_record.views;
        ++ray_record.updates;
    }

    // Mark voxels as viewed; no information updated. Moving from pos_dist to far_dist.
    while (t_x <= t_far_adj || t_y <= t_far_adj || t_z <= t_far_adj)
    {
        try {
            element_ref = &at(current_gidx);
            if (element_ref->views == 0) ++ray_record.first;
            element_ref->setViewUpdateFlag();
        } catch (const std::out_of_range& e) {
            break; // Break if the next update tried to put us out of range.
        }
        if (t_x < t_y && t_x < t_z)
        {
            current_gidx[0] += s_x;
            t_x += dt_x;
        }
        else if (t_y < t_z) // Y is implicitly less than X 
        {
            current_gidx[1] += s_y;
            t_y += dt_y;
        }
        else // Z is implicitly less than both X and Y 
        {
            current_gidx[2] += s_z;
            t_z += dt_z;
        }
        ++ray_record.views;
    }
    return true;
}


} // ForgeScan
