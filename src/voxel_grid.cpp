#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/memory_utils.h>

#include <Eigen/Dense>

#define H5_USE_EIGEN 1
#include <highfive/H5Easy.hpp>


#include <stdexcept>

#include <cmath>

#include <cstdint>

#include <iostream>
#include <iomanip>

#include <fstream>
#include <filesystem>


VoxelGrid::VoxelGrid(double resolution, Eigen::Vector3d lower, Eigen::Vector3d upper, bool round_points_in)
{
    Vector3d span = upper - lower;

    // Validate user inputs
    if (resolution <= 0) throw std::invalid_argument("Resolution must be greater than 0.");
    if ( (span.array() <= 0).any() ) throw std::invalid_argument("Improper upper/lower bounds.");

    this->lower = lower;
    this->upper = upper;
    for (int i = 0; i < 3; ++i)
        this->size[i] = std::ceil(span[i] / resolution);
    this->idx_scale = this->size.cast<double>().array() / span.array();

    size_t vec_len = this->size[0] * this->size[1] * this->size[2];

    this->grid = std::make_shared<std::vector<VoxelElement>>(vec_len, VoxelElement());
    double mem = byte_to_megabytes(vector_capacity(*this->grid));
    if (mem > 100.0)
        std::cout << "Warning, allocated " << mem << " MB for vector grid!" << std::endl;
    this->round_points_in = round_points_in;
}


int VoxelGrid::gidx(const size_t& input, Vector3ui& output) const
{
    static const double sxsy = (double) this->size[0]*this->size[1];

    double copy_idx = (double) input;
    Vector3d temp;

    temp[2] = std::floor(copy_idx / sxsy);

    copy_idx -= temp[2] * sxsy;
    temp[1] = std::floor(copy_idx / this->size[0]);

    copy_idx -= temp[1] * this->size[0];
    temp[0] = copy_idx;

    output = temp.cast<size_t>();

    // To return we check if the temp was out of bounds first. The casting behaviour between a double and
    // unsigned int may be undefined if the cast value cannot be expressed in the destination type.
    // Essentially, negative values in the temp array are impossible to catch in the output alone. See:
    //      https://stackoverflow.com/questions/65012526/
    if ( (temp.array() < 0.0).any() || (temp.array() >= this->size.cast<double>().array()).any() )
        return INVALID_INDEX_ERROR_CODE;
    return this->valid(output) ? 0 : INVALID_INDEX_ERROR_CODE;
}


int VoxelGrid::gidx(const Vector3d& input, Vector3ui& output) const
{
    Vector3d temp = (input - lower).array() * this->idx_scale.array();
    for (int i =0; i < 3; ++i)
    {
        temp[i] = std::round(temp[i]);
        if (this->round_points_in)
        {
            if (temp[i] < 0) temp[i] = 0;
            if (temp[i] >= this->size[i]) temp[i] = this->size[i] - 1;
        }
    }
    output = temp.cast<size_t>();
    return this->valid(output) ? 0 : INVALID_INDEX_ERROR_CODE;
}


int VoxelGrid::sidx(const size_t& input, Vector3d& output) const
{
    Vector3ui gidx;
    this->gidx(input, gidx);
    return this->sidx(gidx, output);
}


int VoxelGrid::sidx(const Vector3ui& input, Vector3d& output) const
{
    output = (input.cast<double>().array() / this->idx_scale.array()) + this->lower.array();

    // NOTE: Check both input and output. The vector idx to space coordinate overload
    //       may provide invalid grid indicies.
    return this->valid(input) && this->valid(output) ? 0 : INVALID_INDEX_ERROR_CODE;
}


int VoxelGrid::vidx(const Vector3d& input, size_t& output) const
{
    Vector3ui gidx;
    this->gidx(input, gidx);
    return this->vidx(gidx, output);
}


int VoxelGrid::vidx(const Vector3ui& input, size_t& output) const
{
    output = ( input[0] ) + ( input[1] * this->size[0] ) + ( input[2] * this->size[0] * this->size[1] );

    // NOTE: Check both input and output. The space coordinate to vector idx overload
    //       may provide invalid grid indicies.
    return this->valid(input) && this->valid(output) ? 0 : INVALID_INDEX_ERROR_CODE;
}


int VoxelGrid::get_6(const Vector3ui& input, std::vector<Vector3ui>& output)
{
    output.clear();
    if (output.capacity() != 6) output.reserve(6);
    for (int i = 0, j = 0; i < 6; ++i)
    {
        output.push_back(input);  // Creates copies of the input
        if (i % 2 == 0)
            output[i][j] += 1;
        else {
            // Unsigned underflow for an index of 0 results in the maximum value that a size_t
            // variable van represent. This is defined behaviour and since this is much larger than
            // any dimension will ever be this will be detected as an invalid index by this->valid 
            //      https://stackoverflow.com/questions/2760502
            output[i][j] -= 1;
            ++j;
        }
    }
    // By checking if the input is on an surface, edge, or corner we can quickly check the validity
    // of the output. A more thorough, test would be checking each derived output. 
    if ( (input.array() == 0).any() || (input.array() >= this->size.array()).any() )
        return INVALID_INDEX_ERROR_CODE;
    return 0;
}


int VoxelGrid::add_linear_space(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const size_t& num, const uint8_t &surface, const uint8_t &line)
{
    // Check that num >= 2

    if (num < 1)
        std::invalid_argument("Must place at least two points on the line segment.");

    // ray = end - start
    Eigen::Vector3d ray, line_space, place;
    ray = end - start;

    // Length of line
    double length = 0;
    for (int i = 0; i < 3; ++i)
        length += std::pow(ray[i], 2);
    length = std::sqrt(length);

    // Normalize the ray
    line_space = ray / (num - 1);

    // Initialize and run the loop
    int success = 0, fails = 0;
    place = start;

    for (int j = 0; j < num - 1; ++j)
    {
        success = this->set(place, line);
        fails += success;

        place += line_space;
    }
    success = this->set(place, surface);
    fails += success;

    return fails;
}


int VoxelGrid::add_line_fast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double& vox_res, const uint8_t &surface, const uint8_t &line)
{
    /* Length of the line segment */
    double dist = 0;
    for (int i =0; i <3; ++i)
        dist += std::pow(end[i] - start [i], 2);
    dist = std::sqrt(dist);

    /* Average spacing as a hack for now; optional different spacing makes things weird here */
    int avg_space = ( this->size[0] + this->size[1] + this->size[2] ) / 3;

    // float

    int num_voxels = dist / avg_space;

    /* Tracks if the line has entered the voxel grid */
    bool entered_grid = false;

    size_t num_vox = dist * vox_res;

    /*
    
    
    for num point in disc. line
        point = start + fraction * (end - start)
    
    
    
    */
    


    return 0;
}


int VoxelGrid::add_sensor(const SimSensorReading& scanner, const uint8_t& surface, const uint8_t& line)
{
    /// @note This has not been fully tested. It could be optimized. And it could mirror or rotate improperly.
    ///       If there are issues this is the place to start debugging.
    /// @note The spacing across the line is set at 300. This is based on the assumption of a camera placed 2.5 meter
    ///       away from a sphere of 0.75 meter radius. All lines should be under 3.25 meters thus this value would place
    ///       at least 1 point in each voxel along the line. TODO: make a more robust method: `add_line_fast`.

    static const Eigen::Vector3d camera_z_axis(0, 0, 1);

    Eigen::MatrixXd copy = scanner.sensor;
    copy.transposeInPlace();  // 3xN matrix

    Eigen::Matrix3d R;
    R = Eigen::Quaterniond().setFromTwoVectors(camera_z_axis, scanner.normal);

    // std::cout << "Camera Z axis:\n" << camera_z_axis << std::endl;
    // std::cout << "scanner position:\n" << scanner.position.transpose() << std::endl;
    // std::cout << "scanner normal:\n" << scanner.normal.transpose() << std::endl;
    // std::cout << "Rotation matrix:\n" << R << std::endl;
    // std::cout << "Prior 10 positions:\n" << scanner.sensor.block(0,0,10,3) << std::endl;

    // APPLY ROTATION
    copy = R*copy;

    // APPLY TRANSLATION
    copy.colwise() += scanner.position;  // Removed previously

    for (size_t i = 0, ncols = copy.cols(); i < ncols; ++i)
    {
        /// TODO: no clue how many points to space. 300 will do for now. See note above.
        this->add_linear_space(scanner.position, copy.col(i), 300, surface, line);
        // std::cout << "[" << i << "]" << " Added final point located at:\n\t" << copy.col(i).transpose() << std::endl;
        // std::cout << "    Sensed point was:\n\t" << scanner.sensor.row(i) << std::endl;
    }
    return 0;
}


void VoxelGrid::save_csv(const std::string& fname) const
{  
    std::ofstream file;

    file.open(fname);
    if (file.is_open() == false) 
    {
        std::cout << "Could not open file!" << std::endl;
        throw std::exception();
    }

    file << std::fixed << std::setprecision(8);
    file << "Test Voxel Grid Format File" << std::endl;
    file << "occupancy value" << std::endl;

    for (const auto& voxel : *this->grid)
        file << voxel.get_view_count() << std::endl;
    file.close();
}


void VoxelGrid::save_hdf5(const std::string& fname) const
{
    throw std::logic_error("Not implemented");
    /*
    HighFive::File file(fname, HighFive::File::ReadWrite | HighFive::File::Truncate);

    file.createDataSet("/data/grid_vector", *this->grid);
    file.createDataSet("/data/size", this->size);

    file.createDataSet("/position/lower", this->lower);
    file.createDataSet("/position/upper", this->upper);
    */
}


void VoxelGrid::load_hdf5(const std::string& fname)
{
    throw std::logic_error("Not implemented");

    /*
    try {
        HighFive::File file(fname, HighFive::File::ReadOnly);
        // Read the data
        // TODO Check the saved data validity or if there is data already in the object
        file.getDataSet("/data/grid_vector").read(*this->grid);
        file.getDataSet("/data/size").read(this->size);

        file.getDataSet("/position/lower").read(this->lower);
        file.getDataSet("/position/upper").read(this->upper);
    } catch (const HighFive::Exception& err) {
        std::cerr << err.what() << std::endl;
    }
    */
}
