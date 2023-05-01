#ifndef FORGESCAN_GRID_TRAVERSAL
#define FORGESCAN_GRID_TRAVERSAL

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/voxel_grid.h>


/// @brief Performs exact voxel traversal for the given ray over the specified time.
/// @param grid The VoxelGrid to traverse.
/// @param rs Ray start position.
/// @param re Ray end position.
/// @returns False if the ray did not intersect the voxel grid. True otherwise.
bool addRayExact(VoxelGrid& grid, const VoxelUpdate& update, const point& rs, const point& re);


/// @brief Adds data to the grid, updating voxels near the sensed point with truncated distance and marking
///        the voxels between the origin and positive truncation as viewed.
/// @param grid The VoxelGrid to add data to.
/// @param origin Origin for the ray.
/// @param sensed Sensed point.
/// @returns False if the ray did not intersect the voxel grid. True otherwise.
bool addRayTSDF(VoxelGrid &grid, const point &origin, const point &sensed);


#endif // FORGESCAN_GRID_TRAVERSAL
