#ifndef FORGESCAN_GRID_TRAVERSAL
#define FORGESCAN_GRID_TRAVERSAL

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/voxel_grid.h>

#include <functional>


/// TODO: Cannot inline because of size? Unclear. Reading the error it looked like a linker error but removing my liberal use of inline
///       did resolve the issue at hand. See the comments on:
///           https://stackoverflow.com/questions/5526461/


/// @brief Checks if an axis-aligned box is intersected with the specified ray and returns be reference the adjusted start/end time for that
///        ray's path through the box.
/// @param lower Lower bound of the axis-aligned box.
/// @param upper Upper bound of the axis-aligned box.
/// @param ray_origin Origin of the ray.
/// @param ray_direction Direction of the ray, a unit vector.
/// @param ts The given start time for walking the ray
/// @param te The given end time for walking the ray
/// @param ts_adj The time when walking the ray will enter the box. Equals ts if the ray begins inside the box.
/// @param te_adj The time when walking the ray will exit the box. Equals te if the ray ends inside the box.
/// @return True if the ray intersects the box over the given start to end time. False if it does not.
bool inline findRayAlignedBoxIntersection(const point& lower,      const point& upper, 
                                          const point& ray_origin, const point& ray_direction,
                                          const double& ts,        const double& te,
                                          double& t_s_adj, double& t_end_adj);


/// @brief Performs exact voxel traversal for the given ray over the specified time.
/// @details Treats the ray as a line parametrized by a variable for time. This time factor describes how
///          long the distance we "walk" from the given start position.
///          Applies the Amanatides-Woo algorithm for fast voxel traversal. See:
///             http://www.cse.yorku.ca/~amana/research/grid.pdf
/// @param grid The VoxelGrid to traverse.
/// @param rs Ray start position.
/// @param re Ray end position.
/// @param operation Function to apply at each voxel that the ray traverses. Default, an empty lambda, does nothing.
/// @param ts Start time for walking the ray (e.g., 0 begins traversal at the given start).
/// @param te End time for walking the ray (e.g., 1 finishes traversal at the given end).
/// @return True if voxels were traversed for the ray. False if the ray did not intersect the grid.
bool addRayExact(VoxelGrid& grid,      const VoxelElementUpdate& update,
                 const point& rs,      const point& re,
                 const std::function<void(const grid_idx&)> operation = [](const grid_idx& gidx){},
                 const double& ts = 0, const double& te = 1);


/// @brief Performs approximate ray addition by a linear spacing of a specific number of points
/// @param grid The VoxelGrid to traverse.
/// @param rs Ray start position.
/// @param re Ray end position.
/// @param num Number of points to add. Must be greater than 2.
/// @param operation Function to apply at each voxel that the ray traverses. Default, an empty lambda, does nothing.
/// @return True.
bool addRayLinspace(VoxelGrid& grid, const VoxelElementUpdate& update,
                    const point& rs, const point& re, const size_t& num,
                    const std::function<void(const grid_idx&)> operation = [](const grid_idx& gidx){});


/// @brief Performs approximate ray addition by spacing
/// @param grid The VoxelGrid to traverse.
/// @param rs Ray start position.
/// @param re Ray end position.
/// @param rr Ray resolution. Percent 0 to 1 relative to the VoxelGrid resolution which determines spacing.
/// @param operation Function to apply at each voxel that the ray traverses. Default, an empty lambda, does nothing.
/// @return True.
bool addRayApprox(VoxelGrid& grid, const VoxelElementUpdate& update,
                  const point& rs, const point& re, const double& rr = 0.9,
                  const std::function<void(const grid_idx&)> operation = [](const grid_idx& gidx){});


#endif // FORGESCAN_GRID_TRAVERSAL
