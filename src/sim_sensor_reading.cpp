#include <point_gen/sim_sensor_reading.h>

#include <highfive/H5Easy.hpp>


SimSensorReading::SimSensorReading(Eigen::Vector3d position, Eigen::Vector3d normal, size_t n, size_t m)
{
    this->position = position;
    this->normal = normal;
    this->n = n;
    this->m = m;

    this->sensor = Eigen::MatrixXd::Zero(n * m, 3);
}


SimSensorReading::SimSensorReading(const std::string& fname)
{
    this->read_sensor_hdf5(fname);
}


SimSensorReading::~SimSensorReading()
{
}


void SimSensorReading::read_sensor_hdf5(const std::string& fname)
{
    try {
        HighFive::File file(fname, HighFive::File::ReadOnly);
        file.getDataSet("/extrinsic/position").read(this->position);
        file.getDataSet("/extrinsic/normal").read(this->normal);
        file.getDataSet("/sensor").read(this->sensor);
    } catch (const HighFive::Exception& err) {
        std::cerr << err.what() << std::endl;
    } 
}


void SimSensorReading::write_sensor_hdf5(const std::string& fname)
{
    try {
        HighFive::File file(fname, HighFive::File::ReadWrite | HighFive::File::Truncate);

        HighFive::DataSet dset_ext_pos = file.createDataSet("/extrinsic/position", this->position);
        HighFive::DataSet dset_ext_nor = file.createDataSet("/extrinsic/normal", this->normal);
        HighFive::DataSet dset_sensor = file.createDataSet("/sensor", this->sensor);

        dset_ext_pos.write(this->position);
        dset_ext_nor.write(this->normal);
        dset_sensor.write(this->sensor);
    } catch (const HighFive::Exception& err) {
        std::cerr << err.what() << std::endl;
    } 
}
