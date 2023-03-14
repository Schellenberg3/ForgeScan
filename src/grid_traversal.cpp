#include <ForgeScan/grid_traversal.h>
#include <iostream>

#define MAX(X, Y) X > Y ? X : Y
#define MIN(X, Y) X < Y ? X : Y
#define MAX_UI64 std::numeric_limits<size_t>::max()

/// OPTIMIZE: There is a chance that Eigen has more overhead for the simple operations we are performing
///           and a custom Vector3 could improve performance.


bool findRayAlignedBoxIntersection(const point& lower,      const point& upper, 
                                   const point& ray_origin, const point& ray_direction,
                                   const double& ts,        const double& te,
                                   double& ts_adj, double& te_adj)
{
    /// @note This checks for several early exit conditions. These are cases where the vector is out of bounds in
    ///       at least one direction over the specified time. The main operation here is computing the time at which
    ///       the ray hits the min/max bounds of the box in each direction. This is why we must have the box aligned
    
    double t_min_y, t_max_y, t_min_z, t_max_z;

    /// OPTIMIZE: Possibly do these one at a time in case we return early and a value is not needed.

    const Vector3d inv_direction = ray_direction.cwiseInverse();
    const Vector3d min_dist = (lower - ray_origin).array() * inv_direction.array();
    const Vector3d max_dist = (upper - ray_origin).array() * inv_direction.array();

    if (inv_direction[0])
    {
        ts_adj = min_dist[0];
        te_adj = max_dist[0];
    } else {
        ts_adj = max_dist[0];
        te_adj = min_dist[0];
    }

    if (inv_direction[1])
    {
        t_min_y = min_dist[1];
        t_max_y = max_dist[1];
    } else {
        t_min_y = max_dist[1];
        t_max_y = min_dist[1];
    }

    if (ts_adj > t_max_y || t_min_y > te_adj) return false;

    if (t_min_y > ts_adj) ts_adj = t_min_y;
    if (t_max_y < te_adj)   te_adj   = t_max_y;

    if (inv_direction[2])
    {
        t_min_z = min_dist[2];
        t_max_z = max_dist[2];
    } else {
        t_min_z = max_dist[2];
        t_max_z = min_dist[2];
    }

    if (ts_adj > t_max_z || t_min_z > te_adj) return false;

    if (t_min_z > ts_adj) ts_adj = t_min_z;
    if (t_max_z < te_adj)   te_adj   = t_max_z;

    return (ts_adj < te && ts < te_adj);
}


bool addRayExact(VoxelGrid& grid,  const VoxelElementUpdate& update,
                 const point& rs,  const point& re, 
                 const std::function<void(const grid_idx&)> operation,
                 const double& ts, const double& te)
{
    double ts_adj, te_adj;

    Vector3d ray = rs - re;
    double   len = ray.norm();
    Vector3d dir = ray / len;
    
    bool valid_segment = findRayAlignedBoxIntersection(grid.lower, grid.upper, rs, dir,
                                                       ts, te, ts_adj, te_adj);
    if (valid_segment == false) return false;

    ts_adj = MAX(ts_adj, ts);
    te_adj = MIN(te_adj, te);

    const point rs_adj = rs + dir * len * ts_adj;
    const point re_adj = rs + dir * len * te_adj;
    Vector3ui current_gidx(0,0,0);
    Vector3ui end_gidx(0,0,0);
    
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
        current_gidx[d] = MAX(0,            std::floor((rs_adj[d] - grid.lower[d]) / grid.resolution));
        end_gidx[d]     = MIN(grid.size[d], std::floor((re_adj[d]   - grid.lower[d]) / grid.resolution));
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
            const size_t previous_gidx = current_gidx[d] - 1;
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

    // Traversal loop
    /// OPTIMIZE: Call operation once on current voxel, then enter the loop. No need to enter the loop/increment in the
    ///           case that the ray starts/ends in the same voxel.
    /// TODO:     Is there an alternative case where traversal time is used instead? Fewer compares?
    grid.set(current_gidx, update);
    while ( (current_gidx.array() != end_gidx.array()).all() ){
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


bool addRayLinspace(VoxelGrid& grid, const VoxelElementUpdate& update,
                    const point& rs, const point& re, const size_t& num,
                    const std::function<void(const grid_idx&)> operation)
{
    if (num < 1) std::invalid_argument("Must place at least two points on the line segment.");

    grid_idx current_idx(0, 0, 0);
    grid_idx previous_idx(MAX_UI64, MAX_UI64, MAX_UI64); 

    point place = rs;

    Vector3d step  = (re - rs) / num;

    Vector3ui current_gidx, previous_gidx;
    for (int j = 0; j < num; ++j)
    {
        grid.toGrid(place, current_idx);
        if ( (current_idx.array() != previous_idx.array()).all() )
            grid.set(place, update);
        previous_idx = current_idx;
        place += step;
    }
    return true;
}


bool addRayApprox(VoxelGrid& grid, const VoxelElementUpdate& update,
                  const point& rs, const point& re, const double& rr,
                  const std::function<void(const grid_idx&)> operation)
{
    if (rr < 0) std::invalid_argument("Resolution must be greater than 0.");

    grid_idx current_idx(0, 0, 0);
    grid_idx previous_idx(MAX_UI64, MAX_UI64, MAX_UI64); 

    point place = rs;

    Vector3d ray  = (re - rs);
    double norm   = ray.norm();
    Vector3d step = (ray / norm) * rr * grid.resolution;

    do {
        grid.toGrid(place, current_idx);
        if ( (current_idx.array() != previous_idx.array()).all() )
            grid.set(place, update);
        previous_idx = current_idx;
        place   += step;
    } while ( (place.array() < re.array()).any() );
    return true;
}
