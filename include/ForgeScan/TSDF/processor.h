#ifndef FORGESCAN_TSDF_PROCESSOR_H
#define FORGESCAN_TSDF_PROCESSOR_H

#include <vector>
#include <functional>

#include "ForgeScan/types.h"
#include "ForgeScan/TSDF/voxel.h"


namespace ForgeScan {
namespace TSDF      {


// Forward definition for the compiler; the Grid class is included in the implementation file.
class Grid;


class Processor {
public:
    /// @brief Pointer to a Grid to perform element-wise processes on.
    Grid *target = nullptr;

public:
    /// @brief Creates a Processor without a target.
    Processor();

    /// @brief Creates a Processor to act of the provided target.
    /// @param target Grid on which perform operations.
    Processor(Grid& target);

    /// @brief Performs an elementwise operation, like erosion or dilation.
    /// @param operation Function to perform. Function must take a constant reference to an voxel.
    void operation(const std::function<void(const index&)>& operation);

    /// @brief Changes what Grid the processor targets for element-wise processes.
    /// @param target Grid on which perform operations.
    inline void setTarget(Grid& new_target);

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


} // namespace TSDF
} // namespace ForgeScan

#endif // FORGESCAN_TSDF_PROCESSOR_H
