#ifndef FORGESCAN_TSDF_VOXEL_H
#define FORGESCAN_TSDF_VOXEL_H

#include "ForgeScan/types.h"

/// Gets the maximum positive value for a float.
#define FLOAT_POSITIVE_MAX std::numeric_limits<float>::max()


namespace ForgeScan {
namespace TSDF      {


/// @brief Storage for data within a Voxel inside of a Grid.
struct Voxel
{
    /// @brief Storage for data used in updating a Voxel inside a Grid.
    struct Update
    {
        voxel_dist dist;
        centrality cent;
        normality  norm;
        density     rho;

        Update(const voxel_dist &dist, const centrality &cent = 0, const normality &norm = 0, const density &rho = 0) :
            dist(dist), cent(cent), norm(norm), rho(rho) {};

        void inline set(const voxel_dist &dist, const centrality &cent = 0, const normality &norm = 0, const density &rho = 0) {
            this->dist = dist;
            this->cent = cent;
            this->norm = norm;
            this->rho  = rho;
        }
    };


    Voxel(const view_count& views = 0, const update_count& updates = 0,
          const voxel_dist& min = 0, const voxel_dist& avg = 0, const voxel_dist& var = 0) {
        this->views = views;
        this->updates = updates;
        this->min = min;
        this->avg = avg;
        this->var = var;
    }

    view_count   views   = 0;
    update_count updates = 0;

    voxel_dist min = FLOAT_POSITIVE_MAX;
    voxel_dist avg = 0;
    voxel_dist var = 0;

    centrality cent = 0;
    normality  norm = 0;
    density    rho  = 0;

    /// @brief Restores the information within a Voxel as if it has never been updated.
    void reset()
    {
        views   = 0;
        updates = 0;

        min = FLOAT_POSITIVE_MAX;
        avg = 0;
        var = 0;

        cent = 0;
        norm = 0;
        rho  = 0;
    }

    /// @brief Updates the information within a Voxel.
    /// @param update The Update with new data.
    /// @warning The `views` attribute of the Voxel will have its MSB set to 1 after this
    ///          function call. This must be set to zero before this count will be correct.
    void update(const Update &update)
    {
        voxel_dist delta = update.dist - avg;

        /// NOTE: Standard deviation is tracked with Welford's Algorithm. What this actually tracks is
        ///       the sum of square differences (from the current mean). Dividing this sum by the number of
        ///       samples (updates) gets the variance. Thus to update we need this annoying multiplication
        ///       and subsequent division. For more, see:
        ///       https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
        var *= updates;

        avg += (delta / ++updates);

        var += ((update.dist - avg) * delta);
        var /= updates;

        // We want to track the value with minimum magnitude, not just the smallest value.
        if ( std::abs(min) > std::abs(update.dist) )
            min = update.dist;

        if (cent < update.cent)
            cent = update.cent;

        if (norm < update.norm)
            norm = update.norm;

        if (rho < update.rho)
            rho = update.rho;

        setViewUpdateFlag();
    }

    /// @brief Marks a Voxel's `view_count` attribute. This signals that at least one ray from a new view
    ///        has updated that Voxel.
    /// @warning When this flag is set, the `view_count` attribute is not accurate when viewed as an integer. It
    ///          should only be incremented when the flag is high. Call `resetViewUpdateFlag` before using this number
    void inline setViewUpdateFlag()  { set_MSB_high(views); }

    /// @brief Resets the flag on a a Voxel's `view_count`. This should be called after the the Voxel's
    ///        `view_count` has been incremented for the latest view so the element is ready to accept new views.
    void inline resetViewUpdateFlag() { set_MSB_low(views); }

    /// @brief Sets the most significant bit of an unsigned 16-bit integer to 1.
    static void inline set_MSB_high(uint16_t &in) { in |= 0x8000; }

    /// @brief Sets the most significant bit of an unsigned 16-bit integer to 0.
    static void inline set_MSB_low(uint16_t &in)  { in &= 0x7FFF; }
};


} // namespace TSDF
} // namespace ForgeScan

#endif // FORGESCAN_TSDF_VOXEL_H
