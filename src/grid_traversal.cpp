#include <ForgeScan/grid_traversal.h>
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

    /// OPTIMIZE: Would using a custom Vector3 class improve perfeormance by reducinc call overhead?
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
    ///       I belive that there may be slight differences with rounding and from voxel resolution.
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


bool addRayLinspace(VoxelGrid& grid, const VoxelUpdate& update,
                    const point& rs, const point& re, const size_t& num)
{
    if (num < 1) std::invalid_argument("Must place at least two points on the line segment.");

    grid_idx current_idx(0, 0, 0);
    grid_idx previous_idx(MAX_UI64, MAX_UI64, MAX_UI64); 

    point place = rs;

    Vector3d step  = (re - rs) / num;

    int sucess = -1;
    for (int j = 0; j < num; ++j)
    {
        grid.toGrid(place, current_idx);
        if ( (current_idx.array() != previous_idx.array()).all() )
            sucess = grid.set(place, update);
        previous_idx = current_idx;
        place += step;
    }
    return true;
}


bool addRayApprox(VoxelGrid& grid, const VoxelUpdate& update,
                  const point& rs, const point& re, const double& rr)
{
    if (rr < 0) std::invalid_argument("Resolution must be greater than 0.");

    grid_idx current_idx(0, 0, 0);
    grid_idx previous_idx(MAX_UI64, MAX_UI64, MAX_UI64); 

    point place = rs;

    Vector3d ray     = (re - rs);
    double ray_norm  = ray.norm();
    Vector3d step    = (ray / ray_norm) * rr * grid.resolution;
    double step_norm = step.norm();

    int num = std::ceil(ray_norm / step_norm);

    int sucess = -1;
    for (int j = 0; j < num; ++j)
    {
        grid.toGrid(place, current_idx);
        if ( (current_idx.array() != previous_idx.array()).all() )
            sucess = grid.set(place, update);
        previous_idx = current_idx;
        place += step;
    }

    return true;
    /*
    do {
        grid.toGrid(place, current_idx);
        if ( (current_idx.array() != previous_idx.array()).all() )
            grid.set(place, update);
        previous_idx = current_idx;
        place   += step;
        /// BUG: This is incorrect. This comparison does not work.
    } while ( (place.array() < re.array()).any() );
    return true;
    */
}
