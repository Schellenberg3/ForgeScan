#ifndef FORGE_SCAN_METRICS_GROUND_TRUTH_EXPERIMENT_VARIANTS_HPP
#define FORGE_SCAN_METRICS_GROUND_TRUTH_EXPERIMENT_VARIANTS_HPP

#include <variant>

#include "ForgeScan/Data/VoxelGrids/Occupancy.hpp"
#include "ForgeScan/Data/VoxelGrids/OccupancyTSDF.hpp"
#include "ForgeScan/Data/VoxelGrids/TSDF.hpp"


namespace forge_scan {
namespace metrics {
namespace ground_truth {


/// @brief Union of shared pointers to the possible types of Voxel Grids that an Occupancy
///        Confusion Metric may compare to a ground truth Occupancy Grid.
typedef std::variant<
    std::shared_ptr<const data::Occupancy>,
    std::shared_ptr<const data::OccupancyTSDF>
>
ExperimentOccupancy;


/// @brief Union of shared pointers to the possible types of Voxel Grids that an TSDF
///        Comparison Metric may compare to a ground truth TSDF Grid.
typedef std::variant<
    std::shared_ptr<const data::TSDF>,
    std::shared_ptr<const data::OccupancyTSDF>
>
ExperimentTSDF;


/// @brief Ensures that the input voxel grid may be cast to one of the variant types in `ExperimentOccupancy`.
/// @param voxel_grid The grid to test.
/// @return An `ExperimentOccupancy` which references the grid.
/// @throws `std::runtime_error` If the Voxel Grid may not be cast to one of the supporting types.
inline ExperimentOccupancy dynamic_cast_to_experimental_occupancy(const std::shared_ptr<const forge_scan::data::VoxelGrid>& voxel_grid)
{
    auto voxel_grid_cast_occupancy = std::dynamic_pointer_cast<const data::Occupancy>(voxel_grid);
    if (voxel_grid_cast_occupancy != nullptr)
    {
        return voxel_grid_cast_occupancy;
    }

    auto voxel_grid_cast_occupancy_tsdf = std::dynamic_pointer_cast<const data::OccupancyTSDF>(voxel_grid);
    if (voxel_grid_cast_occupancy_tsdf != nullptr)
    {
        return voxel_grid_cast_occupancy_tsdf;
    }

    throw std::runtime_error("Failed to cast Voxel Grid to a ExperimentOccupancy type.");
}


/// @brief Ensures that the input voxel grid may be cast to one of the variant types in `ExperimentTSDF`.
/// @param voxel_grid The grid to test.
/// @return An `ExperimentTSDF` which references the grid.
/// @throws `std::runtime_error` If the Voxel Grid may not be cast to one of the supporting types.
inline ExperimentTSDF dynamic_cast_to_experimental_tsdf(const std::shared_ptr<const forge_scan::data::VoxelGrid>& voxel_grid)
{
    auto voxel_grid_cast_tsdf = std::dynamic_pointer_cast<const data::TSDF>(voxel_grid);
    if (voxel_grid_cast_tsdf != nullptr)
    {
        return voxel_grid_cast_tsdf;
    }

    auto voxel_grid_cast_occupancy_tsdf = std::dynamic_pointer_cast<const data::OccupancyTSDF>(voxel_grid);
    if (voxel_grid_cast_occupancy_tsdf != nullptr)
    {
        return voxel_grid_cast_occupancy_tsdf;
    }

    throw std::runtime_error("Failed to cast Voxel Grid to a ExperimentTSDF type.");
}


} // namespace ground_truth
} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_GROUND_TRUTH_EXPERIMENT_VARIANTS_HPP
