#ifndef FORGESCAN_GRID_TRAVERSAL
#define FORGESCAN_GRID_TRAVERSAL

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/sensor_reading.h>

#include <functional>


/// TODO: Cannot inline because of size? Unclear. Reading the error it looked like a linker error but removing my liberal use of inline
///       did resolve the issue at hand. See the comments on:
///           https://stackoverflow.com/questions/5526461/


/// @brief Performs exact voxel traversal for the given ray over the specified time.
/// @details Treats the ray as a line parametrized by a variable for time. This time factor describes how
///          long the distance we "walk" from the given start position.
///          Applies the Amanatides-Woo algorithm for fast voxel traversal. See:
///             http://www.cse.yorku.ca/~amana/research/grid.pdf
/// @param grid The VoxelGrid to traverse.
/// @param rs Ray start position.
/// @param re Ray end position.
/// @param ts Start time for walking the ray (e.g., 0 begins traversal at the given start).
/// @param te End time for walking the ray (e.g., 1 finishes traversal at the given end).
/// @param operation Function to apply at each voxel that the ray traverses. Default, an empty lambda, does nothing.
/// @return True if voxels were traversed for the ray. False if the ray did not intersect the grid.
bool addRayExact(VoxelGrid& grid,      const VoxelUpdate& update,
                 const point& rs,      const point& re,
                 const double& ts = 0, const double& te = 1);


void addRayTSDFandView(VoxelGrid &grid, const point &origin, const point &sensed);


#endif // FORGESCAN_GRID_TRAVERSAL
