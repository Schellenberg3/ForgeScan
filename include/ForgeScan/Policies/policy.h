#ifndef FORGESCAN_POLICIES_BASE_POLICY_H
#define FORGESCAN_POLICIES_BASE_POLICY_H

#include <filesystem>

#include "ForgeScan/types.h"
#include "ForgeScan/TSDF/grid.h"
#include "ForgeScan/TSDF/traversal.h"
#include "ForgeScan/Metrics/sensor_record.h"
#include "ForgeScan/DepthSensor/sensor.h"
#include "ForgeScan/Primitives/scene.h"


namespace ForgeScan {
namespace Policies  {


enum class Type {
    Base,
    OrderedUniform,
    RandomSphere,
    LowDiscrepancy,
    LowDiscrepancyRandomInit
};


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
    virtual bool criteriaMet() const = 0;

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

    /// @brief Saves current state of the policy. The TSDF::Grid is saved in an XDMF format and to its HDF5 file
    ///        the SensorRecord and policy details are also saved.
    /// @param fname File name. Automatically adds ".h5" when writing.
    /// @throws `std::invalid_argument` if there is an issue parsing the file name.
    void save(const std::filesystem::path& fname) const {
        if ( !fname.has_filename() )
            throw std::invalid_argument("[SensorRecord::save] Invalid file name! Could not identify filename.");

        /// Grid::saveXDMF will create the HDF5 file in truncation mode (overwriting any existing contents) such that
        /// SensorRecord::save will write to too truncation mode. Both functions resolve fname into the same file path,
        /// name, and extension.

        grid->saveXDMF(fname);
        sensor_record.save(fname, true);
        derivedClassSavePolicyInfo(fname);
    }

    /// @brief Gets the name of the policy.
    /// @return Name of the policy as a string.
    virtual std::string getName() = 0;

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

    /// @brief Implemented by the derived class to store its parameters with the TSDF::Grid's HDF5 file.
    /// @param fname File name. Automatically adds ".h5" when writing.
    virtual void derivedClassSavePolicyInfo(const std::filesystem::path& fname) const = 0;
};


} // namespace Policies
} // namespace ForgeScan

#endif // FORGESCAN_POLICIES_BASE_POLICY_H
