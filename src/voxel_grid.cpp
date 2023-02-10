#include <point_gen/voxel_grid.h>
#include <point_gen/memory_utils.h>

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


// TODO: REMOVE - replaced by set and its various overloads
int VoxelGrid::set_voxel(const Vector3ui& idx, const uint8_t& val)
{
    if (not this->voxel_in_grid(idx) )
        return 1;
    size_t vec_idx = this->grid_idx_to_vector_idx(idx);
    this->grid[vec_idx] = val;
    return 0;
}


// TODO: REMOVE - replaced by inc and tis various overloads
int VoxelGrid::inc_voxel(const Vector3ui& idx)
{
    if (not this->voxel_in_grid(idx) )
        return 1;
    size_t vec_idx = this->grid_idx_to_vector_idx(idx);
    if (this->grid[vec_idx] < 255)
        this->grid[vec_idx] += 1;
        return 0;
    return 2;
}


VoxelGrid::VoxelGrid(double resolution, Eigen::Vector3d lower, Eigen::Vector3d upper, const uint8_t& init, bool round_points_in)
{
    Vector3d span = upper - lower;

    // Validate user inputs
    if (resolution <= 0) throw std::invalid_argument("Resolution must be greater than 0.");
    if ( (span.array() <= 0).any() ) throw std::invalid_argument("Improper upper/lower bounds.");

    this->lower = lower;
    this->upper = upper;
    for (int i = 0; i < 3; ++i)
        this->space[i] = std::ceil(span[i] / resolution);
    this->idx_scale = space.cast<double>().array() / span.array();

    size_t vec_len = space[0] * space[1] * space[2];
    this->grid.reserve(vec_len);
    double mem = byte_to_megabytes(vector_capacity(this->grid));
    if (mem > 100.0)
        std::cout << "Warning, allocated " << mem << " MB for vector grid!" << std::endl;

    for (size_t i = 0; i < vec_len; ++i)
        this->grid.push_back(init);

    this->round_points_in = round_points_in;
}


int VoxelGrid::gidx(const size_t& input, Vector3ui& output)
{
    static const double sxsy = (double) this->space[0]*this->space[1];

    double copy_idx = (double) input;
    Vector3d temp;

    temp[2] = std::floor(copy_idx / sxsy);

    copy_idx -= temp[2] * sxsy;
    temp[1] = std::floor(copy_idx / this->space[0]);

    copy_idx -= temp[1] * this->space[0];
    temp[0] = copy_idx;

    output = temp.cast<size_t>();
    if ( (temp.array() < 0.0).any() || (temp.array() >= this->space.cast<double>().array()).any() )
        return -1;
    return this->valid(output) ? 0 : -1;
}


int VoxelGrid::gidx(const Vector3d& input, Vector3ui& output)
{
    Vector3d temp = (input - lower).array() * this->idx_scale.array();
    for (int i =0; i < 3; ++i)
    {
        temp[i] = std::round(temp[i]);
        if (this->round_points_in)
        {
            if (temp[i] < 0) temp[i] = 0;
            if (temp[i] >= this->space[i]) temp[i] = this->space[i] - 1;
        }
    }
    output = temp.cast<size_t>();
    return this->valid(output) ? 0 : -1;
}


int VoxelGrid::sidx(const size_t& input, Vector3d& output)
{
    Vector3ui gidx;
    this->gidx(input, gidx);
    return this->sidx(gidx, output);
}


int VoxelGrid::sidx(const Vector3ui& input, Vector3d& output)
{
    output = (input.cast<double>().array() / this->idx_scale.array()) + this->lower.array();

    // NOTE: Check both input and output. The vector idx to space coordinate overload
    //       may provide invalid grid indicies.
    return this->valid(input) && this->valid(output) ? 0 : -1;
}


int VoxelGrid::vidx(const Vector3d& input, size_t& output)
{
    Vector3ui gidx;
    this->gidx(input, gidx);
    return this->vidx(gidx, output);
}


int VoxelGrid::vidx(const Vector3ui& input, size_t& output)
{
    output = ( input[0] ) + ( input[1] * this->space[0] ) + ( input[2] * this->space[0] * this->space[1] );

    // NOTE: Check both input and output. The space coordinate to vector idx overload
    //       may provide invalid grid indicies.
    return this->valid(input) && this->valid(output) ? 0 : -1;
}


// TODO: REMOVE
int VoxelGrid::add_point(const Eigen::Vector3d& point, const uint8_t& val)
{
    Vector3ui idx;
    int result = space_to_grid_idx(point, idx);
    if (result != 0)
        return 1;
    return this->set_voxel(idx, val);
}


// TODO: REMOVE - replaced by inc and its various overloads
int VoxelGrid::inc_point(const Eigen::Vector3d& point)
{
    Vector3ui idx;
    int result = space_to_grid_idx(point, idx);
    if (result != 0)
        return 1;
    return this->inc_voxel(idx);
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
        success = this->add_point(place, line);
        fails += success;

        place += line_space;
    }
    success = this->add_point(place, surface);
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
    int avg_space = ( this->space[0] + this->space[1] + this->space[2] ) / 3;

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


void VoxelGrid::save_csv(const std::string& fname)
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

    for (auto voxel = this->grid.begin(); voxel != this->grid.end(); ++voxel)
    {
        file << (int)*voxel << std::endl;
    }

    file.close();
}


void VoxelGrid::save_hdf5(const std::string& fname)
{
    HighFive::File file(fname, HighFive::File::ReadWrite | HighFive::File::Truncate);

    file.createDataSet("/data/grid_vector", this->grid);
    file.createDataSet("/data/spacing", this->space);

    file.createDataSet("/position/lower", this->lower);
    file.createDataSet("/position/upper", this->upper);
}


void VoxelGrid::load_hdf5(const std::string& fname)
{
    try {
        HighFive::File file(fname, HighFive::File::ReadOnly);

        // Read the data 
        /* TODO Check the saved data validity or if there is data already in the object  */
        file.getDataSet("/data/grid_vector").read(this->grid);
        file.getDataSet("/data/spacing").read(this->space);

        file.getDataSet("/position/lower").read(this->lower);
        file.getDataSet("/position/upper").read(this->upper);
    } catch (const HighFive::Exception& err) {
        std::cerr << err.what() << std::endl;
    }
}


// TODO: REMOVE - replaced by gidx for space and vector transformations
Vector3ui VoxelGrid::vector_idx_to_grid_idx(const size_t& vector_idx)
{
    Vector3ui grid_idx;
    size_t copy_idx = vector_idx;
    size_t sxsy = this->space[0]*this->space[0];

    grid_idx[2] = std::floor(copy_idx / sxsy);

    copy_idx -= grid_idx[2] * sxsy;
    grid_idx[1] = std::floor(copy_idx / this->space[0]);

    copy_idx -= grid_idx[1] * this->space[0];
    grid_idx[0] = copy_idx;

    return grid_idx;
}

// TODO: REMOVE - replaced by gidx for space and vector transformations
int VoxelGrid::space_to_grid_idx(const Eigen::Vector3d& space_xyz, Vector3ui& grid_idx)
{
    /// NOTE: The logic here could likely be made more efficient. Not a major concern
    ///       right now but potential future work.
    int success = 0;
    double float_idx = 0, max_idx = 0;
    for (int i = 0; i < 3; ++i)
    {
        max_idx = this->space[i] - 1.0;
        float_idx = std::round( ( space_xyz[i] - this->lower[i] ) * this->idx_scale[i] );

        if (this->round_points_in)
        {
            if (float_idx < 0.)
                float_idx = 0.;
            else if (float_idx > max_idx)
                float_idx = max_idx;
        } else {
            if (float_idx < 0.0)
            {
                if (-1 < float_idx)
                    float_idx = 0.;
                else
                    success = 1;
            }
            else if (max_idx < float_idx)  
            {
                if (float_idx < max_idx + 1.0)
                    float_idx = max_idx;
                else
                    success = 1;
            }
        }

        /// Negatives are messing this up big time!
        /// This is why the success return variable is needed.
        grid_idx[i] = (size_t) float_idx;
    }
    return success;
}


// TODO: REMOVE - replaced by at and its various overloads
uint8_t VoxelGrid::space_at(const Eigen::Vector3d& space_xyz)
{
    Vector3ui grid_idx;
    int result = space_to_grid_idx(space_xyz, grid_idx);
    size_t vec_idx = this->grid_idx_to_vector_idx(grid_idx);
    
    if (vec_idx >= this->grid.size() || result != 0)
        return 0;
    
    return this->grid[vec_idx];
}


// TODO: REMOVE - replaced by valid and its various overloads
bool VoxelGrid::space_in_grid(const Eigen::Vector3d& space_xyz)
{
    bool test = true;
    for (int i = 0; i < 3; ++i)
    {
        if (not (this->lower[i] <= space_xyz[i] < this->upper[i]) )
        {
            test = false;
            break;
        }
    }
    return test;
}

// TODO: REMOVE - replaced by at and its various overloads
uint8_t VoxelGrid::voxel_at(const Vector3ui& voxel_idx)
{
    size_t vec_idx = this->grid_idx_to_vector_idx(voxel_idx);
    if (vec_idx > this->grid.size())
        return 0;
    return this->grid[vec_idx];
}

// TODO: REMOVE - replaced by valid and its various overloads
bool VoxelGrid::voxel_in_grid(const Vector3ui& voxel_idx)
{
    bool test = true;
    for (int i = 0; i < 3; ++i)
    {
        if (not (0 <= voxel_idx[i] < this->space[i]) )
        {
            test = false;
            break;
        }
    }
    return test;
}


void VoxelGrid::get_6_connect_vector_list_idx(const Vector3ui& grid_idx, std::vector<size_t>& connected)
{
    connected.clear();
    connected.reserve(6);

    size_t v_idx = this->grid_idx_to_vector_idx(grid_idx);
    size_t dz = this->space[0] * this->space[1];

    connected.push_back(v_idx - 1);
    connected.push_back(v_idx + 1);
    connected.push_back(v_idx + this->space[0]);
    connected.push_back(v_idx - this->space[0]);
    connected.push_back(v_idx + dz);
    connected.push_back(v_idx - dz);
}
