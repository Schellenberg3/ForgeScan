#include <ForgeScan/grid_traversal.h>
#include <ForgeScan/sim_sensor_reading.h>

#include <iostream>

#define MAX(X, Y) X > Y ? X : Y
#define MIN(X, Y) X < Y ? X : Y
#define MAX_UI64 std::numeric_limits<size_t>::max()


/// @brief Verifies that a given ray does intersect the given axis-aligned box.
/// @details This checks several early exit conditions and may not return accurate the adjusted start and end times
///          in the case that the box is not intersected.
///          The values for ts and te are in multiples of the unit-vector direction.
/// @param lower Lower bound for the box.
/// @param upper Upper bound for the box.
/// @param ray_origin Origin of the ray.
/// @param ray_direction Direction of the ray.
/// @param ts Start time for walking the array. A value of zero indicates starting at the origin.
/// @param te End time for walking the array.
/// @param ts_adj An adjusted start time for when the begins intersecting the box.
/// @param te_adj An adjusted end time for when the ray finishes intersecting the box.
/// @return True if the ray intersects the box in the region given by `ts` and `te`
/// @warning Does not verify if `ray_direction` is a unit vector.
/// @warning Does not verify if the upper point is greater than the lower point.
/// @warning Does not check that `te` is less than `ts`.
bool findRayAlignedBoxIntersection(const point& lower,      const point& upper, 
                                   const point& ray_origin, const point& ray_direction,
                                   const double& ts,        const double& te,
                                   double& ts_adj, double& te_adj)
{
    /// @note This checks for several early exit conditions. These are cases where the vector is out of bounds in
    ///       at least one direction over the specified time. The main operation here is computing the time at which
    ///       the ray hits the min/max bounds of the box in each direction. This is why we must have the box aligned
    
    double t_min_y, t_max_y, t_min_z, t_max_z;

    const Vector3d inv_direction = ray_direction.cwiseInverse();
    const Vector3d min_dist = (lower - ray_origin).array() * inv_direction.array();
    const Vector3d max_dist = (upper - ray_origin).array() * inv_direction.array();

    if (inv_direction[0] >= 0)
    {
        ts_adj = min_dist[0];
        te_adj = max_dist[0];
    } else {
        ts_adj = max_dist[0];
        te_adj = min_dist[0];
    }

    if (inv_direction[1] >= 0)
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

    if (inv_direction[2] >= 0)
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


bool addRayExact(VoxelGrid& grid,  const VoxelUpdate& update,
                 const point& rs,  const point& re,
                 const double& ts, const double& te)
{
    /// @note: This method works quite well and is very fast. However it tends to miss the final
    ///        1 to 3 voxels. The exit condition is a bit imperfect. Maybe some boolean flags rather
    ///        than comparisons will correct this. But this is a minor error at the moment.

    /// OPTIMIZE: Would using a custom Vector3 class improve performance by reducing call overhead?
    ///           Eigen is nice. But it wraps a lot of un-needed functionality, especially for just
    ///           a collection of 3 elements. 

    Vector3d ray = re - rs;
    double   len = ray.norm();
    Vector3d dir = ray / len;

    // We must update the precents given as an input to multiples of the unit vector direction. 
    double ts_units = ts * len;
    double te_units = te * len;

    // We declare adjusted start and end times (in multiples of the unit vector) so the
    // findRayAlignedBoxIntersection method can adjust our times so we only look at valid voxels.
    double ts_adj, te_adj;

    // Early exit for segments outside of the grid or for equal start/end points. 
    bool valid_segment = findRayAlignedBoxIntersection(grid.lower, grid.upper, rs, dir,
                                                       ts_units, te_units, ts_adj, te_adj);
    if (valid_segment == false)
    {
        // Invalid cases could be completely outside the grid or an invalid line.
        // We try to update the starting point in case we have the case of an invalid line because
        // the start/end points are the same. If this is inside the grid we get the proper update.
        int res = grid.set(rs, update);
        return res == 0;
    }

    // From the adjusted times and given unit-vector multiple times we want to find the max start time
    // and the min end time. This is 
    ts_adj = MAX(ts_adj, ts_units);
    te_adj = MIN(te_adj, te_units);

    const point rs_adj = rs + dir * ts_adj;
    const point re_adj = rs + dir * te_adj;

    Vector3ui current_gidx(0,0,0);

    // For debugging it is helpful to know the end index but it is not used in the traversal algorithm.
    // Vector3ui end_gidx(0,0,0);  
    
    /// [voxel] For each direction, as the ray is traversed, we either increment (+1) or decrement (-1) the index
    /// when we choose to step in that direction.
    int s_x = 0, s_y = 0, s_z = 0;

    /// [time] For each direction, we record the cumulative time spent traversing that direction. We always step in
    /// the direction that has been traversed the least.
    double t_x = 0, t_y = 0, t_z = 0;

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
        current_gidx[d] = MAX(0, std::floor((rs_adj[d] - grid.lower[d]) / grid.resolution));
        // See note above. Helpful for debugging but not needed for the algorithm.
        // end_gidx[d]     = MIN(grid.size[d], std::floor((re_adj[d] - grid.lower[d]) / grid.resolution));
        if (dir[d] > 0)
        {
            s_d  = 1;
            dt_d = grid.resolution / dir[d];
            t_d  = ts_adj + (grid.lower[d] + current_gidx[d] * grid.resolution - rs_adj[d]) / dir[d];
        }
        else if (dir[d] != 0)
        {
            s_d  = -1;
            dt_d = -1 * grid.resolution / dir[d];
            const int previous_gidx = current_gidx[d] - 1;  // size_t casued an error here if current == 0
            t_d  = ts_adj + (grid.lower[d] + previous_gidx * grid.resolution - rs_adj[d]) / dir[d];
        }
        else
        {
            s_d  = 0;
            dt_d = te_adj;
            t_d  = te_adj;
        }
    };

    initTraversalInfo(s_x, t_x, dt_x, 0);
    initTraversalInfo(s_y, t_y, dt_y, 1);
    initTraversalInfo(s_z, t_z, dt_z, 2);

    /// NOTE: The exact start/end voxels hit tend to be a bit off from what might be "exact".
    ///       I believe that there may be slight differences with rounding and from voxel resolution.
    ///       But I will return to this another time.
    grid.set(current_gidx, update); // update current element befor entering the loop.
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
        // operation(current_gidx);
        grid.set(current_gidx, update);
    }
    return true;
}


void addSensor(VoxelGrid &grid, const SimSensorReading &sensor)
{
    const Eigen::Vector3d camera_z_axis(0, 0, 1);

    Eigen::MatrixXd copy = sensor.sensor;
    copy.transposeInPlace();  // 3xN matrix

    Eigen::Matrix3d R;
    R = Eigen::Quaterniond().setFromTwoVectors(camera_z_axis, sensor.normal);

    // APPLY ROTATION
    copy = R*copy;

    // APPLY TRANSLATION
    copy.colwise() += sensor.position;  // Removed previously

    for (int i = 0, num_pts = sensor.m * sensor.n; i < num_pts; ++i)
    {
        addRayTSDFandView(grid, sensor.position, copy.col(i));
    }

    for (auto& ve : *(grid.grid))
    {
        if (ve.views >> 15)
        {   // Checks if the MSB of the views is set to 1 and 
            if (ve.views != 0xFFFF) ++ve.views;   // Caps updates and prevents rollover at 0x7FFF (32767 views.)
            resetViewUpdateFlag(ve);
        }
    }
}


/// @note: This method works quite well and is very fast. However it tends to miss the final 1 to 3 voxels. The exit condition
///        is a bit imperfect. Maybe some boolean flags rather than comparisons will correct this. But this is a minor error at the moment.
void addRayTSDFandView(VoxelGrid &grid, const point &origin, const point &sensed)
{
    /// [Distance] Adjusted traversal distances that are inside the grid. Given a grid's tuncation distance the adjusted values follow:
    ///  NEGATIVE TRUCATION DISTANCE <= t_neg_adj <= t_pos_adj <= POSITIVE TRUCATION DISTANCE,
    ///  t_pos_adj <= t_far_adj
    ///  In some conditions the adjusted negative value may be greater than zero (and the adjusted positive value less than zero). 
    double t_pos_adj = 0, t_neg_adj = 0, t_far_adj = 0;
    Vector3ui current_gidx(0,0,0);
    VoxelUpdate update(0);

    Vector3d ray    = origin - sensed;
    double t_far    = ray.norm();
    Vector3d normal = ray / t_far;

    /// [Voxel] For each direction, as the ray is traversed, we either increment (+1) or decrement (-1) the index
    /// when we choose to step in that direction.
    int s_x = 0, s_y = 0, s_z = 0;

    /// [Distance] For each direction, we record the cumulative traveled distance that direction. We always step in
    /// the direction that has been traversed the least.
    double t_x = 0, t_y = 0, t_z = 0;

    /// [Distance] For each direction, we find amount of travel required to move the distance of one voxel edge length in
    /// that direction. For each movement in a direction, this is added to the cumulative total for that direction.
    double dt_x = 0,  dt_y = 0,  dt_z = 0;

    // Early exit for segments outside of the grid or for equal start/end points. 
    bool intersects_grid = findRayAlignedBoxIntersection(grid.lower, grid.upper, sensed, normal, grid.neg_trunc_dist, t_far, t_neg_adj, t_far_adj);

    t_far_adj = MIN(t_far_adj, t_far);
    t_pos_adj = MIN(t_far_adj, grid.pos_trunc_dist);
    t_neg_adj = MAX(t_neg_adj, grid.neg_trunc_dist);

    bool correct_traversal_direction = t_neg_adj < t_far_adj;
    if (intersects_grid == false || correct_traversal_direction == false)
        return;

    const point neg_dist = sensed + normal * t_neg_adj;

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
        current_gidx[d] = MAX(0, std::floor((neg_dist[d] - grid.lower[d]) / grid.resolution - 1));
        if (normal[d] > 0)
        {
            s_d  = 1;
            dt_d = grid.resolution / normal[d];
            t_d  = t_neg_adj + abs( (grid.lower[d] + current_gidx[d] * grid.resolution - neg_dist[d]) / normal[d] );
        }
        else if (normal[d] != 0)
        {
            s_d  = -1;
            dt_d = -1 * grid.resolution / normal[d];
            int previous_gidx = current_gidx[d] - 1;
            t_d  = t_neg_adj + abs( (grid.lower[d] + previous_gidx * grid.resolution - neg_dist[d]) / normal[d] );
        }
        else
        {
            s_d  = 0;
            dt_d = t_far_adj;
            t_d  = t_far_adj;
        }
    };

    initTraversalInfo(s_x, t_x, dt_x, 0);
    initTraversalInfo(s_y, t_y, dt_y, 1);
    initTraversalInfo(s_z, t_z, dt_z, 2);

    // Perform updates withing the truncation distance. Moving from neg_dist to pos_dist.
    update.dist = t_neg_adj;
    while (t_x <= t_pos_adj || t_y <= t_pos_adj || t_z <= t_pos_adj)
    {
        grid.set(current_gidx, update);
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
    }

    // Mark voxels as viewed; no information updated. Moving from pos_dist to far_dist.
    while (t_x <= t_far_adj || t_y <= t_far_adj || t_z <= t_far_adj)
    {
        setViewUpdateFlag( grid.at(current_gidx) );
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
    }
}
