#ifndef FORGESCAN_VOXEL_ELEMENT_H
#define FORGESCAN_VOXEL_ELEMENT_H

#include <limits>
#include <cstdint>
#include <stdexcept>


class GridProcessor;  // Promise to compiler that the GridProcessor class will be defined later


// Effectively sets the max at 65,353 views with just 2 Bytes.
typedef uint16_t voxel_views;

// Truncated distance where positive values are outside the part, negative values
// are inside the part, and the surface is implicitly represented by the 0 level. 
typedef float voxel_distance;

// Centrality score for the voxel. Score increases with the further a voxel is from
// the sensor's principle axis. 
typedef float voxel_centrality;

// Normality score relating to how closely the surface's normal aligns with that of
// the principle axis in a given scan. 
typedef float voxel_normality;

// Density score for the voxel. Score is related to how many sensed points fall with
// that voxel in a given scan.
typedef float voxel_density;


/// @brief Container for updates to a voxel. Using pointers we can indicate if information
///        does not exist (and the should not be updated) by setting it to nullptr.
/// @note  Does not contain a new voxel_view; incrementing the count is implicit.
struct VoxelElementUpdate {
    voxel_distance*   new_dist    = nullptr;
    voxel_centrality* new_cent    = nullptr;
    voxel_normality*  new_norm    = nullptr;
    voxel_density*    new_density = nullptr;

    /// @brief Constructor for distance-only update.
    /// @param new_dist Pointer to new voxel_distance variable.
    VoxelElementUpdate(voxel_distance* new_dist) :
        new_dist(new_dist), new_cent(nullptr), new_norm(nullptr), new_density(nullptr) { }
};


/// @brief A compact container for attributes relevant to each voxel within a grid.
///
/// @details It is important for this class to be relatively small. At present it should be just 18 Bytes.
///          However, it seems to be 20 Bytes at the moment. See the todo note below for details.
///
/// @todo Despite view_count being of type unsigned short (2 Bytes) and all 4 other members being of type
///       float (4 Bytes), the overall class size is 20. Not 18 as it should be. This is annoying. Perhaps it
///       has to do with my compiler settings. But it is not of immediate importance.
class VoxelElement {
    friend class GridProcessor;

    /// @brief Records number of times a voxel has been updated with new information.
    voxel_views view_count = 0;
    
    /// @brief Truncated distance, minimum, from the voxel to the object surface
    voxel_distance min_dist = 0;

    /// @brief Running average of the truncated signed distances.
    voxel_distance avg_dist = 0;

    /// @brief Running aggregation of squared distances for Welford's algorithm.
    voxel_distance agg_sq_dist = 0;

    /// @brief Centrality score for the voxel based on its views.  
    voxel_centrality cent = 0;

    /// @brief Normal score for way the voxel has been viewed.
    voxel_normality norm = 0;

    /// @brief Measurement density of sensed points within the voxel based on its views.
    voxel_density density = 0;

    /// @brief Resets all members to zero.
    void inline reset() {
        view_count = 0;
        min_dist = 0;
        avg_dist = 0;
        agg_sq_dist = 0;
        cent = 0;
        norm = 0;
        density = 0;
    }

public:
    VoxelElement(){};

    /// @param view_count Number of views, typically 0 at the beginning. 
    /// @param dist Initial truncated distance.
    /// @param cent Initial centrality score.
    /// @param density  Initial density score.
    /// @param norm Initial normality score.
    VoxelElement(voxel_views view_count, voxel_distance distance = 0, voxel_centrality cent = 0,
                 voxel_normality norm = 0, voxel_density density = 0) :
                 view_count(view_count), min_dist(distance), avg_dist(distance),
                 cent(cent), density(density), norm(norm) { }

    /// @brief Gets count for the number of views the voxel has.
    /// @return Number of views.
    voxel_views inline get_view_count() const { return view_count; }

    /// @brief Gets the voxels minimum truncated distance to the measured surface.
    /// @return Minimum truncated distance to surface.
    /// @note Positive values are away from the surface.
    voxel_distance inline get_min_distance() const { return min_dist; }

    /// @brief Gets the voxels average truncated to the measured surface.
    /// @return Average truncated distance to surface.
    /// @note Positive values are away from the surface.
    voxel_distance inline get_avg_distance() const { return avg_dist; }

    /// @brief Gets the standard deviation of the voxels truncated distance to the measured surface.
    /// @return Standard deviation for the  average truncated distance to surface.
    voxel_distance inline get_std_distance() const { return agg_sq_dist / view_count; }

    /// @brief Gets the centrality value for the voxel.
    /// @return Centrality value.
    voxel_centrality inline get_centrality() const { return cent; }

    /// @brief Gets the normality information about the voxel.
    /// @return Normality score.
    voxel_normality inline get_normality() const { return norm; }

    /// @brief Gets the density of sensed points within the voxel.
    /// @return Sensor measurement density.
    voxel_density inline get_density() const { return density; }

    /// @brief Increments the view counter without changing other information.
    void inline inc_views() { ++view_count; }

    /// @brief Updates the truncated distance by taking the minimum distance.
    /// @param new_dist New truncated distance.
    void inline update_min_dist(const voxel_distance& new_dist) {
        if (new_dist < min_dist)
            min_dist = new_dist;
    }

    /// @brief Updates the truncated distance with a running average and running standard deviation.
    /// @param new_dist New truncated distance.
    /// @note Applies Welford's online algorithm. See:
    ///       https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
    void inline update_avg_std_dist(const voxel_distance& new_dist) {
        float delta = (new_dist - avg_dist);
        avg_dist += delta / (view_count + 1);
        agg_sq_dist += (new_dist - avg_dist) * delta;  // Divide by N for the standard deviation
    }

    /// @brief Updates the centrality score by taking the minimum centrality value.
    /// @param new_dist New centrality score.
    void inline update_cent(const voxel_centrality& new_cent) {
        if (new_cent < cent)
            cent = new_cent;
    }

    /// @brief Updates the normality score
    /// @param new_norm New normality score.
    /// @warning NOT IMPLEMENTED.
    /// @throws std::logic_error
    void inline update_norm(const voxel_normality& new_norm) {
        std::logic_error("Function not yet implemented");
    }

    /// @brief Updates the density score by taking the maximum density.
    /// @param new_density New normality score.
    void inline update_density(const voxel_density& new_density) {
        if (new_density < density)
            density = new_density;
    }
};


/// @brief Updates the specified VoxelElement with the provided VoxelElementUpdate.
/// @param element Element to target with the updates.
/// @param update  Values to update the target members with. Update members pointing
///                to nullptr will be skipped.
/// @note Always increments the view count of the targeted VoxelElement.
void update_voxel_element(VoxelElement& element, const VoxelElementUpdate& update);


#endif // FORGESCAN_VOXEL_ELEMENT_H
