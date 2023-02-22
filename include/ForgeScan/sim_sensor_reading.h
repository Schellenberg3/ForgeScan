#ifndef FORGESCAN_SIM_SENSOR_READING_H
#define FORGESCAN_SIM_SENSOR_READING_H

#include <Eigen/Geometry>

#include <string>

/// @brief Container for simulated sensor readings from depth cameras and laser scanners 
class SimSensorReading
{
/// TODO: Proper separation of public/private later 
// private:
public:
    // Extrinsic position information about the camera
    Eigen::Vector3d position, normal;
    
    // A Nx3 list of the N points seen by the camera; order does not matter
    Eigen::MatrixXd sensor;

    // Dimension of the NxM image; may be Nx1 for laser scans
    size_t n, m;


// public:
    /// @brief Constructs a camera based on the provided informaiton
    /// @param position Cartesian location of the sensor
    /// @param normal Normal view for the sensor
    /// @param n Number of vertical pixel (camera) or points (laser scanner)
    /// @param m Number og horizontal pixel (camera) or 1 (laser scanner)
    /// @note TODO incorporate more intrinsics for FOV or other properties
    SimSensorReading(Eigen::Vector3d position, Eigen::Vector3d normal, size_t n, size_t m = 1);


    /// @brief Constructor to load from a HDF5 file
    /// @param fname Name of the file
    SimSensorReading(const std::string& fname);


    ~SimSensorReading();


    /// @brief Reads the HDF5 file and sets the sensor properties appropriately
    /// @param fname Name of the file
    void read_sensor_hdf5(const std::string& fname);


    /// @brief Writes the HDF5 file and sets the sensor properties appropriately
    /// @param fname Name of the file
    void write_sensor_hdf5(const std::string& fname);


    /// @brief Getter method for a sensor
    /// @param n Pixel row n (camera) or point n (laser scanner)
    /// @param m Pixel column m (camera) or 1 (laster scanner)
    /// @return the value at that location
    double at(const int& n, const int& m = 1) 
        {return sensor(n * m);}


    /// @brief Setter method for a sensor
    /// @param val Value to set that position to
    /// @param n Pixel row n (camera) or point n (laser scanner)
    /// @param m Pixel column m (camera) or 1 (laster scanner)
    /// @return the value at that location
    void set(const double& val, const int& n, const int& m = 1)
        { sensor(n * m) = val;}
};

#endif // FORGESCAN_SIM_SENSOR_READING_H
