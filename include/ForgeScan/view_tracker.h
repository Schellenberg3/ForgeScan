#ifndef FORGESCAN_VIEW_TRACKER_H
#define FORGESCAN_VIEW_TRACKER_H

#include <cmath>
#include <vector>
#include <stdexcept>

#include <ForgeScan/forgescan_types.h>


namespace ForgeScan {


/// @brief Record for a specific view.
struct RayRecord 
{
    size_t updates = 0;
    size_t views = 0;
    size_t first = 0;

    double max_variance_update = -1;

    /// @brief Resets all members to zero.
    void reset()
    {
        updates = 0;
        views = 0;
        first = 0;
        max_variance_update = -1;
    }
};

/// @brief Record of information about a sensor.
///        Each ray contributes its own information to this from a RayRecord object.    
struct SensorRecord
{
    /// @brief Transformation, relative to the VoxelGrid.
    extrinsic pose;

    size_t total_updates = 0;
    size_t total_views   = 0;
    size_t total_first   = 0;

    size_t size = 0;

    double percent_hit    = 0;
    double percent_viewed = 0;

    double max_variance_update = 0;

    /// @brief Default constructor. No pose information provided, assumed to be identity. No size, assumed to be 0.
    SensorRecord() { pose.setIdentity(); }

    /// @brief Default constructor with pose information. 
    /// @param pose Pose of the sensor relative to the VoxelGrid.
    /// @param size Number of points in the depth sensor.
    SensorRecord(const extrinsic& pose, const size_t& size) : pose(pose), size(size) { }

    /// @brief Convenience for updating with data from each ray.
    /// @param ray_data Data from the latest ray.
    SensorRecord& operator+=(const RayRecord& ray_data)
    {
        // throw std::logic_error("Function not fully implemented.");
        if (ray_data.views == 0) // Ray missed.
            return *this;
        
        // Increase the count of total viewed voxels and the count of individual rays that viewed at least one voxel.
        total_views += ray_data.views;
        ++percent_viewed;

        // Increase the count of total updated voxels and the count of individual rays that updated at least one voxel.
        if (ray_data.updates > 0) {
            total_updates += ray_data.updates;
            ++percent_hit;  //
        }

        total_first += ray_data.first;
        max_variance_update = std::max(max_variance_update, ray_data.max_variance_update);

        return *this;
    }
};

/// @brief Records pose of each view and statistics about about how it changed the VoxelGrid.
class ViewTracker
{
public:
    /// @brief Ordered list of each View's pose and metrics about the information it added to the VoxelGrid.
    std::vector<SensorRecord> record;

    /// @brief Constructor. Pre-allocates space for 20 views.
    ViewTracker() { record.reserve(20); }

    /// @brief Adds the SensorRecord. Performing any final calculations one all RayRecords have been added to it.
    /// @param sensor_record Data to be added.
    void add(SensorRecord& sensor_record) {
        // Turn counts of hits/views into percentages based on sensor size.
        sensor_record.percent_hit    /= sensor_record.size;
        sensor_record.percent_viewed /= sensor_record.size;

        record.push_back(sensor_record);
    }
};


} // ForgeScan

#endif // FORGESCAN_VIEW_TRACKER_H
