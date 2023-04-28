#include <ForgeScan/voxel_grid.h>
#include <ForgeScan/memory_utils.h>

#include <Eigen/Dense>

#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>

#include <stdexcept>

#include <cmath>

#include <cstdint>

#include <iostream>
#include <iomanip>

#include <fstream>
#include <filesystem>


VoxelGrid::VoxelGrid(double resolution, point lower, point upper, bool round_points_in)
{
    Vector3d span = upper - lower;

    // Validate user inputs
    if (resolution <= 0) throw std::invalid_argument("Resolution must be greater than 0.");
    if ( (span.array() <= 0).any() ) throw std::invalid_argument("Improper upper/lower bounds.");

    this->resolution = resolution;
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


int VoxelGrid::toGrid(const vector_idx& input, Vector3ui& output) const
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


int VoxelGrid::toGrid(const point& input, grid_idx& output) const
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


int VoxelGrid::toPoint(const vector_idx& input, point& output) const
{
    grid_idx gidx;
    this->toGrid(input, gidx);
    return this->toPoint(gidx, output);
}


int VoxelGrid::toPoint(const grid_idx& input, point& output) const
{
    output = (input.cast<double>().array() / this->idx_scale.array()) + this->lower.array();

    // NOTE: Check both input and output. The vector idx to space coordinate overload
    //       may provide invalid grid indicies.
    return this->valid(input) && this->valid(output) ? 0 : INVALID_INDEX_ERROR_CODE;
}


int VoxelGrid::toVector(const point& input, vector_idx& output) const
{
    if ( !valid(input) )
        return INVALID_INDEX_ERROR_CODE;
    grid_idx gidx;
    int valid = this->toGrid(input, gidx);
    return this->toVector(gidx, output);
}


int VoxelGrid::toVector(const grid_idx& input, vector_idx& output) const
{
    output = ( input[0] ) + ( input[1] * this->size[0] ) + ( input[2] * this->size[0] * this->size[1] );

    // NOTE: Check both input and output. The space coordinate to vector idx overload
    //       may provide invalid grid indicies.
    return this->valid(input) && this->valid(output) ? 0 : INVALID_INDEX_ERROR_CODE;
}


int VoxelGrid::get_6(const grid_idx& input, std::vector<grid_idx>& output)
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


void VoxelGrid::saveCSV(const std::string& fname) const
{  
    std::ofstream file;
    file.exceptions( std::ofstream::failbit | std::ofstream::badbit );
    try
    {
        file.open(fname + ".csv");
        file << std::fixed << std::setprecision(8);
        file << "Voxel Grid CSV Format\n";
        file << "Updates, Views, Min, Avg, Var, Cent, Norm, Rho\n";
        for (const auto& voxel : *this->grid)
            file << voxel.updates << ", "
                << voxel.views   << ", "
                << voxel.min     << ", "
                << voxel.avg     << ", "
                << voxel.var     << ", "
                << voxel.cent    << ", "
                << voxel.norm    << ", "
                << voxel.rho     << ",\n";
        file.close();
    }
    catch (std::ofstream::failure e)
    {
        std::cerr << "Encountered error in VoxelGrid::saveCSV. See message:\n" << e.what();
    }
}


HighFive::CompoundType create_compound_VoxelElement() {
    return {
        {"views",  HighFive::AtomicType<view_count>{}},
        {"updates",  HighFive::AtomicType<update_count>{}},
        
        {"min",  HighFive::AtomicType<voxel_dist>{}},
        {"avg",  HighFive::AtomicType<voxel_dist>{}},
        {"var",  HighFive::AtomicType<voxel_dist>{}},

        {"cent", HighFive::AtomicType<centrality>{}},
        {"norm", HighFive::AtomicType<normality>{}},
        {"rho", HighFive::AtomicType<density>{}}
    };
}


HIGHFIVE_REGISTER_TYPE(VoxelElement, create_compound_VoxelElement)


void VoxelGrid::saveHDF5(const std::string& fname) const
{
    try
    {
        HighFive::File file(fname + ".h5", HighFive::File::Truncate);

        auto VoxelElementType = create_compound_VoxelElement();
        VoxelElementType.commit(file, "VoxelElement");

        auto g1 = file.createGroup("VoxelGrid");

        g1.createDataSet("VoxelData", *grid);

        g1.createAttribute("Resolution", resolution);
        g1.createAttribute("Upper", upper);
        g1.createAttribute("Lower", lower);
    }
    catch (const HighFive::Exception& err)
    {
        std::cerr << err.what() << std::endl;
    } 
}


void VoxelGrid::loadHDF5(const std::string& fname)
{
    try
    {
        HighFive::File file(fname + ".h5", HighFive::File::ReadOnly);

        auto g1 = file.getGroup("VoxelGrid");

        auto dset = g1.getDataSet("VoxelData");
        dset.read(*grid);

        auto res = g1.getAttribute("Resolution");
        res.read(resolution);

        auto ub = g1.getAttribute("Upper");
        ub.read(upper);

        auto lb = g1.getAttribute("Lower");
        lb.read(lower);
    }
    catch (const HighFive::Exception& err)
    {
        std::cerr << err.what() << std::endl;
    }
}


void VoxelGrid::saveXDMF(const std::string &fname) const
{
    const int num_element = grid->size();
    VoxelElement const *voxel_ref;  // 
    try
    {
        HighFive::File file(fname + ".h5", HighFive::File::Truncate);

        auto VoxelElementType = create_compound_VoxelElement();
        VoxelElementType.commit(file, "VoxelElement");

        auto g1 = file.createGroup("VoxelGrid");

        std::vector<update_count> vec_updates;
        std::vector<view_count>   vec_views;
        std::vector<voxel_dist>   vec_min;
        std::vector<voxel_dist>   vec_avg;
        std::vector<voxel_dist>   vec_var;
        std::vector<centrality>   vec_cent;
        std::vector<normality>    vec_norm;
        std::vector<density>      vec_rho;

        vec_updates.reserve(num_element);
        vec_views.reserve(num_element);
        vec_min.reserve(num_element);
        vec_avg.reserve(num_element);
        vec_var.reserve(num_element);
        vec_cent.reserve(num_element);
        vec_norm.reserve(num_element);
        vec_rho.reserve(num_element);

        for (int i = 0; i < num_element; ++i)
        {
            voxel_ref = &grid->at(i);
            vec_updates.push_back(voxel_ref->updates);
            vec_views.push_back(voxel_ref->views);
            vec_min.push_back(voxel_ref->min);
            vec_avg.push_back(voxel_ref->avg);
            vec_var.push_back(voxel_ref->var);
            vec_cent.push_back(voxel_ref->cent);
            vec_norm.push_back(voxel_ref->norm);
            vec_rho.push_back(voxel_ref->rho);
        }

        g1.createDataSet("Updates",    vec_updates);
        g1.createDataSet("Views",      vec_views);
        g1.createDataSet("Minimum",    vec_min);
        g1.createDataSet("Average",    vec_avg);
        g1.createDataSet("Variance",   vec_var);
        g1.createDataSet("Centrality", vec_cent);
        g1.createDataSet("Normality",  vec_norm);
        g1.createDataSet("Density",    vec_rho);

        g1.createAttribute("Resolution", resolution);
        g1.createAttribute("Upper", upper);
        g1.createAttribute("Lower", lower);

        writeXDMF(fname);
    }
    catch (const HighFive::Exception& err) {
        std::cerr << err.what() << std::endl;
    } 
}


void VoxelGrid::writeXDMF(const std::string &fname) const
{
    std::string hdf5_fname = fname + ".h5";

    const int num_element = grid->size();

    point lower_zyx = lower;
    lower_zyx.reverseInPlace();

    Vector3ui size_1 = size.array() + 1;
    size_1.reverseInPlace();

    std::ofstream file;
    file.exceptions( std::ofstream::failbit | std::ofstream::badbit );
    try
    {
        file.open(fname + ".xdmf");
        file << std::fixed << std::setprecision(8);
        file << 
        "<?xml version=\"1.0\"?>\n"
        "<!DOCTYPE Xdmf SYSTEM \"Xdmf.dtd\"[]>\n"
        "<Xdmf xmlns:xi=\"http://www.w3.org/2003/XInclude\" Version=\"2.2\">\n"
        " <Domain>\n"
        "  <!-- *************** START OF VOXELGRID *************** -->\n"
        "  <Grid Name=\"VOXELGRID\" GridType=\"Uniform\">\n"
        "    <Geometry Type=\"ORIGIN_DXDYDZ\">\n"
        "      <Topology TopologyType=\"3DCoRectMesh\" Dimensions=\"" << size_1.transpose() << " \"></Topology>\n" <<
        "      <!-- Origin  Z, Y, X -->\n"
        "      <DataItem Format=\"XML\" Dimensions=\"3\">" << lower_zyx.transpose() << "</DataItem>\n" <<
        "      <!-- DxDyDz (Spacing/Resolution) Z, Y, X -->\n"
        "      <DataItem Format=\"XML\" Dimensions=\"3\">" << resolution << " " << resolution << " " << resolution << "</DataItem>\n"
        "    </Geometry>\n"

        //  Updates
        "    <Attribute Name=\"Updates\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Int\" Precision=\"2\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Updates\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Views
        "    <Attribute Name=\"Views\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Int\" Precision=\"2\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Views\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Minimum
        "    <Attribute Name=\"Minimum\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Minimum\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Average
        "    <Attribute Name=\"Average\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Average\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Variance
        "    <Attribute Name=\"Variance\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Variance\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Centrality
        "    <Attribute Name=\"Centrality\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Centrality\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Normality
        "    <Attribute Name=\"Normality\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Normality\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Density
        "    <Attribute Name=\"Density\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_element << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/VoxelGrid/Density\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        " </Grid>\n"
        " <!-- *************** END OF VOXELGRID *************** -->\n"
        " </Domain>\n"
        "</Xdmf>\n"
        << std::endl;
    }
    catch (std::ofstream::failure e)
    {
        std::cerr << "Encountered error in VoxelGrid::writeXDMF. See message:\n" << e.what();
    }
}
