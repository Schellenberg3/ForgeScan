#ifndef FORGESCAN_TSDF_TRAVERSAL_H
#define FORGESCAN_TSDF_TRAVERSAL_H

#include "ForgeScan/types.h"
#include "ForgeScan/TSDF/grid.h"


namespace ForgeScan {
namespace TSDF      {


/// @brief Declaration for the implementation which all overloads of addRayUpdate reference.
/// @param grid   Grid to traverse.
/// @param rs     Start of the ray, relative to the grid's frame.
/// @param re     End of the ray, relative to the grid's frame.
/// @param update Update to apply at each traversed voxel.
/// @return True if at least one voxel was updated by the ray.
/// @note   Does nothing, returning false, if rs and re are exactly the same point.
bool implementAddRayUpdate(Grid& grid, const point& rs, const point& re, const Voxel::Update& update = Voxel::Update(1, 0, 0, 0));

/// @brief Declaration for the implementation which all overloads of addRayTSDF reference.
/// @param grid   Grid to update, with the TSDF parameters to use.
/// @param origin Origin of the ray, relative to the grid's frame.
/// @param sensed Sensed point, relative to the grid's frame.
/// @param ray_metrics[out] Output variable with metrics about the TSDF updates.
/// @return True if at least one voxel was traversed by the ray.
/// @note   Does nothing, returning false, if rs and re are exactly the same point.
bool implementAddRayTSDF(Grid& grid, const point &origin, const point &sensed, Metrics::Ray& ray_metrics = Metrics::Ray());

/// @brief Applied the provided Update to each voxel on the line between the specified points.
/// @param grid   Grid to traverse.
/// @param rs Ray start position.
/// @param re Ray end position.
/// @param update Update to apply, default value has distance set to 1 and all other values set to 0.
/// @param extr   Extrinsic reference frame of the ray coordinates. Default is the world reference frame.
/// @return True if at least one voxel was updated by the ray.
/// @note   Does nothing, returning false, if rs and re are exactly the same point.
bool addRayUpdate(Grid& grid, const point& rs, const point& re,
                  const Voxel::Update& update = Voxel::Update(1, 0, 0, 0),
                  const extrinsic& extr = extrinsic::Identity())
{
    point rs_this = grid.toThisFromOther(rs, extr), re_this = grid.toThisFromOther(re, extr);
    bool res = implementAddRayUpdate(grid, rs_this, re_this, update);
    grid.updateViewCount();
    return res;
}

/// @brief Applied the provided Update to each voxel on the line between the specified points.
/// @param grid   Grid to traverse.
/// @param rs Start position for all rays.
/// @param re List of ray end positions.
/// @param update Update to apply, default value has distance set to 1 and all other values set to 0.
/// @param extr   Extrinsic reference frame of the ray coordinates. Default is the world reference frame.
/// @return True if at least one voxel was updated by a ray.
bool addRayUpdate(Grid& grid, const point& rs, const point_list& re,
                  const Voxel::Update& update = Voxel::Update(1, 0, 0, 0),
                  const extrinsic& extr = extrinsic::Identity())
{
    const size_t n = re.cols();
    point rs_this = grid.toThisFromOther(rs, extr);
    point_list re_this = grid.toThisFromOther(re, extr);
    bool res = false;
    for (size_t i = 0; i < n; ++i) {
        res |= implementAddRayUpdate(grid, rs_this, re_this.col(n), update);
    }
    grid.updateViewCount();
    return res;
}

/// @brief Applied the provided Update to each voxel on the line between the specified points.
/// @param grid   Grid to traverse.
/// @param rs List of ray start positions.
/// @param re List of ray end positions.
/// @param update Update to apply, default value has distance set to 1 and all other values set to 0.
/// @param extr   Extrinsic reference frame of the ray coordinates. Default is the world reference frame.
/// @return True if at least one voxel was updated by a ray.
/// @throws `std::invalid_argument` If rs and re have a different number of columns.
bool addRayUpdate(Grid& grid, const point_list& rs, const point_list& re,
                  const Voxel::Update& update = Voxel::Update(1, 0, 0, 0),
                  const extrinsic& extr = extrinsic::Identity())
{
    const size_t n = re.cols();
    if (n != rs.cols()) {
        throw std::invalid_argument("[ForgeScan::TSDF::addRayUpdate] Input start and end point_list must be of the same size.");
    }
    point_list rs_this = grid.toThisFromOther(rs, extr), re_this = grid.toThisFromOther(re, extr);
    bool res = false;
    for (size_t i = 0; i < n; ++i) {
        res |= implementAddRayUpdate(grid, rs_this.col(n), re_this.col(n), update);
    }
    grid.updateViewCount();
    return res;
}

/// @brief Applied the provided Update to each voxel seen by the depth sensor.
/// @param grid   Grid to traverse.
/// @param sensor Sensor to add.
/// @param update Update to apply, default value has distance set to 1 and all other values set to 0.
/// @return True if at least one voxel was updated by a ray from the sensor.
bool addSensorUpdate(Grid& grid, const DepthSensor::Sensor&sensor, const Voxel::Update& update = Voxel::Update(1, 0, 0, 0))
{
    /// Get the sensor's measured points, relative to the sensor frame, then transform
    /// these points from the sensor frame to this Grid's frame.
    point_list points = sensor.getAllPositions();
    grid.toThisFromOther(points, sensor);

    /// Get the sensor's position (for the start of each ray) relative to the world frame, then
    /// transform this to the Grid's frame.
    point sensor_pose = sensor.extr.translation();
    grid.toThisFromWorld(sensor_pose);

    /// The points variables is a 3xN matrix, add each one.
    bool res = false;
    auto n = points.cols();
    for (int i = 0; i < n; ++i) {
        res |= implementAddRayUpdate(grid, sensor_pose, points.col(i), update);
    }
    grid.updateViewCount();

    return res;
}

/// @brief Adds the ray between the origin and sensed points as a TSDF update.
/// @param grid   Grid to update, with the TSDF parameters to use.
/// @param origin Origin of the ray.
/// @param sensed Sensed point around which the TSDF is updated.
/// @param extr   Extrinsic reference frame of the ray coordinates. Default is the world reference frame.
/// @param ray_metrics[out] Output variable with metrics about the TSDF updates.
/// @return True if at least one voxel was traversed by the ray.
bool addRayTSDF(Grid& grid, const point &origin, const point &sensed,
                const extrinsic& extr = extrinsic::Identity(),
                Metrics::Ray& ray_metrics = Metrics::Ray())
{
    point origin_this = grid.toThisFromOther(origin, extr), sensed_this = grid.toThisFromOther(sensed, extr);
    bool res = implementAddRayTSDF(grid, origin_this, sensed_this, ray_metrics);
    grid.updateViewCount();
    return res;
}

/// @brief Adds the ray between the origin and sensed points as a TSDF update.
/// @param grid   Grid to update, with the TSDF parameters to use.
/// @param origin Origin of the ray.
/// @param sensed List of sensed points around which the TSDF is updated.
/// @param extr   Extrinsic reference frame of the ray coordinates. Default is the world reference frame.
/// @param ray_metrics[out] Output variable with metrics about the TSDF updates.
/// @return True if at least one voxel was traversed by a ray.
bool addRayTSDF(Grid& grid, const point &origin, const point_list &sensed,
                const extrinsic& extr = extrinsic::Identity(),
                Metrics::Sensor& sensor_metrics = Metrics::Sensor())
{
    const size_t n = sensed.cols();
    point      origin_this = grid.toThisFromOther(origin, extr);
    point_list sensed_this = grid.toThisFromOther(sensed, extr);
    Metrics::Ray ray_metrics;
    bool res = false;
    for (size_t i = 0; i < n; ++i) {
        res |= implementAddRayTSDF(grid, origin_this, sensed_this.col(n), ray_metrics);
        sensor_metrics += ray_metrics;
        ray_metrics.reset();
    }
    grid.updateViewCount();
    return res;
}

/// @brief Adds the ray between the origin and sensed points as a TSDF update.
/// @param grid   Grid to update, with the TSDF parameters to use.
/// @param origin List of origins for rays.
/// @param sensed List of sensed points around which the TSDF is updated.
/// @param extr   Extrinsic reference frame of the ray coordinates. Default is the world reference frame.
/// @param ray_metrics[out] Output variable with metrics about the TSDF updates.
/// @return True if at least one voxel was traversed by a ray.
/// @throws `std::invalid_argument` If origin and sensed have a different number of columns.
bool addRayTSDF(Grid& grid, const point_list &origin, const point_list &sensed,
                const extrinsic& extr = extrinsic::Identity(),
                Metrics::Sensor& sensor_metrics = Metrics::Sensor())
{
    const size_t n = sensed.cols();
    if (n != origin.cols()) {
        throw std::invalid_argument("[ForgeScan::TSDF::addRayTSDF] Input origin and sensed point_list must be of the same size.");
    }
    point_list origin_this = grid.toThisFromOther(origin, extr), sensed_this = grid.toThisFromOther(sensed, extr);
    Metrics::Ray ray_metrics;
    bool res = false;
    for (size_t i = 0; i < n; ++i) {
        ray_metrics.reset();
        res |= implementAddRayTSDF(grid, origin_this.col(n), sensed_this.col(n), ray_metrics);
        sensor_metrics += ray_metrics;
    }
    grid.updateViewCount();
    return res;
}

/// @brief Adds the measurements recorded by the depth sensor.
/// @param grid   Grid to update, with the TSDF parameters to use.
/// @param sensor Sensor to add.
/// @param sensor_record[out] Output variable to record metrics about the TSDF updates from this sensor.
/// @return True if at least one voxel was traversed by a ray from the sensor.
bool addSensorTSDF(Grid& grid, const DepthSensor::Sensor&sensor,
                   Metrics::SensorRecord& sensor_record = Metrics::SensorRecord())
{
    /// Get the sensor's measured points, relative to the sensor frame, then transform
    /// these points from the sensor frame to this Grid's frame.
    point_list points = sensor.getAllPositions();
    grid.toThisFromOther(points, sensor);

    /// Get the sensor's position (for the start of each ray) relative to the world frame, then
    /// transform this to the Grid's frame.
    point sensor_pose = sensor.extr.translation();
    grid.toThisFromWorld(sensor_pose);

    /// Set up tracking objects for the SensorRecord.
    Metrics::Sensor sensor_metrics( grid.getTransformationTo(sensor), sensor.intr->size() );
    Metrics::Ray ray_metrics;

    /// The points variables is a 3xN matrix, add each one.
    bool res = false;
    auto n = points.cols();
    for (int i = 0; i < n; ++i) {
        ray_metrics.reset();
        res |= implementAddRayTSDF(grid, sensor_pose, points.col(i), ray_metrics);
        sensor_metrics += ray_metrics;
    }
    sensor_record.add(sensor_metrics);
    grid.updateViewCount();

    return res;
}


} // namespace TSDF
} // namespace ForgeScan

#endif // FORGESCAN_TSDF_TRAVERSAL_H
