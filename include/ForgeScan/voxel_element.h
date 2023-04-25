#ifndef FORGESCAN_VOXEL_ELEMENT_H
#define FORGESCAN_VOXEL_ELEMENT_H

#include <ForgeScan/forgescan_types.h>

/// Gets the maximum positive value for a float.
#define FLOAT_POSITIVE_MAX std::numeric_limits<float>::max()


/// @brief Storage for data within a Voxel inside of a VoxelGrid.
struct VoxelElement
{
    view_count   views   = 0;
    update_count updates = 0;
    
    voxel_dist min = FLOAT_POSITIVE_MAX;
    voxel_dist avg = 0;
    voxel_dist var = 0;

    centrality cent = 0;
    normality  norm = 0;
    density    rho  = 0;
};


/// @brief Storage for data used in updating a Voxel inside a VoxelGrid. 
struct VoxelUpdate
{
    voxel_dist dist;
    centrality cent;
    normality  norm;
    density     rho;

    VoxelUpdate(const voxel_dist &dist, const centrality &cent = 0,
                const normality &norm = 0, const density &rho = 0) :
        dist(dist), cent(cent), norm(norm), rho(rho) {};
    
    void inline set(const voxel_dist &dist, const centrality &cent = 0,
        const normality &norm = 0, const density &rho = 0)
    {
        this->dist += 1; // = dist;
    }
};


/// @brief Sets the most significant bit of an unsigned 16-bit integer to 1.
void inline set_MSB_high(uint16_t &in) { in |= 0x8000; }


/// @brief Sets the most significant bit of an unsigned 16-bit integer to 0.
void inline set_MSB_low(uint16_t &in)  { in &= 0x7FFF; }


/// @brief Marks a VoxelElement's `view_count` attribute. This signals that at least one ray from a new view
///        has updated that VoxelElement.
/// @param in The VoxelElement's `views` attribute.
/// @warning When this flag is set, the `view_count` attribute is not accurate when viewed as an integer. It
///          should only be incremented when the flag is high. Call `resetViewUpdateFlag` before using this number
void inline setViewUpdateFlag(view_count &in)  { set_MSB_high(in); }


/// @brief Marks a VoxelElement's `view_count` attribute. This signals that at least one ray from a new view
///        has updated that VoxelElement.
/// @param in The VoxelElement to flag.
/// @warning When this flag is set, the `view_count` attribute is not accurate when viewed as an integer. It
///          should only be incremented when the flag is high. Call `resetViewUpdateFlag` before using this number
void inline setViewUpdateFlag(VoxelElement &in)  { set_MSB_high(in.views); }


/// @brief Resets the flag on a a VoxelElement's `view_count`. This should be called after the the VoxelElement's
///        `view_count` has been incremented for the latest view so the element is ready to accept new views.
/// @param in The VoxelElement to un-flag.
void inline resetViewUpdateFlag(VoxelElement &in) { set_MSB_low(in.views); }


/// @brief Resets the flag on a a VoxelElement's `view_count`. This should be called after the the VoxelElement's
///        `view_count` has been incremented for the latest view so the element is ready to accept new views.
/// @param in The VoxelElement's `views` attribute.
void inline resetViewUpdateFlag(view_count &in) { set_MSB_low(in); }


/// @brief Updates the information within a VoxelElement.
/// @param target The VoxelElement to update with new information.
/// @param update The VoxelUpdate with the new data.
/// @warning The `views` attribute of the VoxelElement will have its MSB set to 1 after this
///          function call. This must be set to zero before this count will be correct. The 
void inline updateVoxel(VoxelElement& target, const VoxelUpdate &update)
{   
    voxel_dist delta = update.dist - target.avg;

    /// NOTE: Standard deviation is tracked with Welford's Algorithm. What this actually tracks is
    ///       the sum of square differences (from the current mean). Dividing this sum by the number of
    ///       samples (updates) gets the variance. Thus to update we need this annoying multiplication
    ///       and subsequent division. For more, see: 
    ///       https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
    target.var *= target.updates;

    target.avg += (delta / ++target.updates);

    target.var += ((update.dist - target.avg) * delta);
    target.var /= target.updates;

    // We want to track the value with minimum magnitude, not just the smallest value.
    if ( std::abs(target.min) > std::abs(update.dist) )
        target.min = update.dist;

    if (target.cent < update.cent)
        target.cent = update.cent;

    if (target.norm < update.norm)
        target.norm = update.norm;

    if (target.rho < update.rho)
        target.rho = update.rho;

    setViewUpdateFlag(target.views);
}

/// @brief Restores the information within a VoxelElement as if it has never been updated.
/// @param target The VoxelElement to reset.
void inline resetVoxel(VoxelElement& target)
{
    target.views   = 0;
    target.updates = 0;
    
    target.min = FLOAT_POSITIVE_MAX;
    target.avg = 0;
    target.var = 0;

    target.cent = 0;
    target.norm = 0;
    target.rho  = 0;
}


#endif // FORGESCAN_VOXEL_ELEMENT_H
