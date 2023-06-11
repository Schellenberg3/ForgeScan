#ifndef FORGESCAN_GRID_PROCESSOR_H
#define FORGESCAN_GRID_PROCESSOR_H

#include <vector>
#include <functional>

#include <ForgeScan/forgescan_types.h>
#include <ForgeScan/voxel.h>
#include <ForgeScan/voxel_grid.h>


namespace ForgeScan {


class VoxelGrid;  // Promise to compiler that the VoxelGrid class will be defined later


class GridProcessor {
public:
    /// @brief Pointer to a VoxelGrid to perform element-wise processes on.
    VoxelGrid *target = nullptr;

public:
    /// @brief Creates a GridProcessor without a target.
    GridProcessor();

    /// @brief Creates a GridProcessor to act of the provided target.
    /// @param target VoxelGrid on which perform operations.
    GridProcessor(VoxelGrid& target);

    /// @brief Performs an elementwise operation, like erosion or dilation.
    /// @param operation Function to perform. Function must take a constant reference to an voxel.
    void operation(const std::function<void(const index&)>& operation);

    /// @brief Changes what VoxelGrid the processor targets for element-wise processes.
    /// @param target VoxelGrid on which perform operations.
    inline void setTarget(VoxelGrid& new_target);

private:
    /// @brief Temporary vector that is the destination for changes when running an element-wise process.
    std::vector<Voxel> temp;

private:
    /// @brief Resets the temporary vector to the provided value.
    void inline resetTempVector() { for (auto& voxel : temp) voxel.reset(); }

    /// @brief Swaps the temporary and target voxel voxel vectors.
    inline void swap();

    /// @brief Erodes the voxel in the temporary vector at the provided location.
    /// @param voxel Location to erode.
    /// @param n Number of un-sensed (zero) neighbors. If n or greater are un-sensed, then the voxel is set to 0.
    void erode(const index& voxel, const int& n);


    /// @brief Dilates the voxel in the temporary vector at the provided location.
    /// @param voxel Location to dilate.
    /// @param n Number of sensed (non-zero) neighbors. If n or greater are sense, then the voxel is set to 1.
    void dilate(const index& voxel, const int& n);
};


} // ForgeScan

#endif // FORGESCAN_GRID_PROCESSOR_H
