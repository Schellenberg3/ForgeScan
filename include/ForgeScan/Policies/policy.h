#ifndef FORGESCAN_POLICIES_BASE_POLICY_H
#define FORGESCAN_POLICIES_BASE_POLICY_H

#include "ForgeScan/types.h"
#include "ForgeScan/TSDF/grid.h"
#include "ForgeScan/TSDF/traversal.h"
#include "ForgeScan/Metrics/sensor_record.h"
#include "ForgeScan/DepthSensor/sensor.h"
#include "ForgeScan/Primitives/scene.h"


namespace ForgeScan {
namespace Policies  {


/// @brief Abstract policy class. Manages data capture by observing a TSDF Grid and positioning a DepthSensor.
class Policy {
public:
    /// @brief TSDF Grid of the ongoing reconstruction.
    TSDF::Grid *const grid;

    /// @brief Sensor for collecting data from a scene.
    DepthSensor::Sensor *const sensor;

    /// @brief The scene being observed by the DepthSensor and reconstructed by the TSDF Grid.
    const Primitives::Scene *const scene;

    /// @brief Record of what sensors were added by the policy and in what order.
    Metrics::SensorRecord sensor_record;

    /// @brief Checks if the policies stopping criteria are met.
    /// @return True if the criteria are met. False else.
    virtual bool criteriaMet()  = 0;

    /// @brief Sets the camera at the next position for the sensor based on the policies decision making strategy.
    virtual void nextPosition() = 0;

    void run() {
        while (!criteriaMet()) {
            preRunLoopCall();
            nextPosition();
            sensor->resetDepth();
            sensor->image(*scene);
            TSDF::addSensorTSDF(*grid, *sensor, sensor_record);
            postRunLoopCall();
        }
    }

protected:
    /// @brief Protected constructor for derived classes to call.
    /// @param grid   Grid for the ongoing reconstruction.
    /// @param sensor Sensor for collecting data from the scene and adding it to the grid.
    /// @param scene  Scene to be reconstructed.
    Policy(TSDF::Grid& grid, DepthSensor::Sensor& sensor, const Primitives::Scene& scene) :
        grid(&grid), sensor(&sensor), scene(&scene)
        { }

    /// @brief Points the DepthSensor at the center of the TSDF Grid.
    void orientSensorToGridCenter() { sensor->orientPrincipleAxis(grid->getCenter()); }

    /// @brief Runs before the start of each loop of the run method.
    virtual void preRunLoopCall()  { };

    /// @brief Runs before the end of each loop of the run method.
    virtual void postRunLoopCall() { };

};


} // namespace Policies
} // namespace ForgeScan

#endif // FORGESCAN_POLICIES_BASE_POLICY_H
