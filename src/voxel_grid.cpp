#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <cmath>

/// Eigen is used when saving/loading from HDF5 files.
#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>

#include <ForgeScan/voxel_grid.h>
#include <ForgeScanUtils/memory_utils.h>


/// @brief Helper for VoxelGrid::saveHDF5 to inform HighFive of the datatypes represented in the `voxel_element_vector`.
static HighFive::CompoundType create_compound_VoxelElement() {
    return {
        {"views",  HighFive::AtomicType<ForgeScan::view_count>{}},
        {"updates",  HighFive::AtomicType<ForgeScan::update_count>{}},
        
        {"min",  HighFive::AtomicType<ForgeScan::voxel_dist>{}},
        {"avg",  HighFive::AtomicType<ForgeScan::voxel_dist>{}},
        {"var",  HighFive::AtomicType<ForgeScan::voxel_dist>{}},

        {"cent", HighFive::AtomicType<ForgeScan::centrality>{}},
        {"norm", HighFive::AtomicType<ForgeScan::normality>{}},
        {"rho", HighFive::AtomicType<ForgeScan::density>{}}
    };
}
HIGHFIVE_REGISTER_TYPE(ForgeScan::VoxelElement, create_compound_VoxelElement)


namespace ForgeScan {


/// @note The methods for `VoxelGrid::implementAddRayExact` and `VoxelGrid::implementAddRayTSDF` are defined in `voxel_grid_traversal.cpp`.

/// TODO: VoxelGrid::loadHDF5 cannot function as designed. See not in function. Move this to its own stand-alone function.


/// @brief A long helper function that modifies the input VoxelGridProperties to be valid for a voxel grid.
/// @note  It preferences keeping the user's resolution and adjusting the grid size then dimensions as needed.
///        It may expand the grid size and dimensions, but cannot shrink it.
/// @param input_properties User's provided input properties.
/// @return VoxelGridProperties which are valid and attempt to stay true to the user's input properties.
/// @throws `std::invalid_argument` if the resolution and dimensions cannot be determined -- where the resolution
///         is non-positive and any dimension is non-positive in any direction.
static VoxelGridProperties setVoxelGridProperties(const VoxelGridProperties& input_properties)
{
    VoxelGridProperties adjusted_properties = input_properties;

    if(std::isnan(adjusted_properties.max_dist)|| std::isnan(adjusted_properties.max_dist))
        throw std::invalid_argument("setVoxelGridProperties:: Input properties minimum and maximum truncation distances cannot be NAN.");

    /// Flags for each key voxel grid attribute. Set to true if the input values MUST be changed. However,
    /// even if these are false, the adjusted properties may differ slightly from the input properties
    bool set_resolution = input_properties.resolution <= 0,
         set_grid_size  = (input_properties.grid_size.array() == 0).any(),
         set_dimensions = (input_properties.dimensions.array() <= 0).any();

    if (set_resolution)
    {   /// If the adjusted resolution MUST BE SET...
        if (!set_dimensions)
        {   /// If the adjusted dimension MUST BE SIMILAR TO THE INPUT... 
            if (set_grid_size)
            {   /// If the adjusted grid MUST BE SET... 
                /// The resolution IS SET so 100 voxels would span the average of all 3 dimensions.
                adjusted_properties.resolution = input_properties.dimensions.sum() / 300;
            }
            else
            {   /// If the adjusted grid size MUST BE SIMILAR TO THE INPUT...
                /// The resolution IS SET so the most restrictive dimension of the grid size given the dimensions is met.
                Vector3d voxel_cuboid_size = input_properties.dimensions.array() / input_properties.grid_size.cast<double>().array();
                adjusted_properties.resolution = voxel_cuboid_size.minCoeff();
            }

            /// The grid size IS SET or IS ADJUSTED, based on the resolution, to enclose the input dimensions.
            adjusted_properties.grid_size = (input_properties.dimensions / adjusted_properties.resolution).array().ceil().cast<size_t>();

            /// The dimensions ARE ADJUSTED to match the grid size and dimension.
            adjusted_properties.dimensions = adjusted_properties.grid_size.cast<double>() * adjusted_properties.resolution;

            /// Sanity checks to ensure that grid size and dimensions ARE SIMILAR TO THE INPUT.
            /// We want to ensure that we have only expanded the grid_size and dimensions.
            if (!set_grid_size) { 
                assert((adjusted_properties.grid_size.array() >= input_properties.grid_size.array()).all() &&
                        "Adjustments should never shrink the input grid size in any direction.");
            }
            assert((adjusted_properties.dimensions.array() >= input_properties.dimensions.array()).all() &&
                    "Adjustments should never shrink the input grid dimensions in any direction.");

            /// Resolution IS SET, grid size IS SET or IS ADJUSTED, and dimensions ARE ADJUSTED.
            /// We can now exit function to return valid adjusted_properties.
        }
        else
        {   /// No valid cases for:
            ///     adjusted resolution MUST BE SET and adjusted dimensions MUST BE SET for any provided grid size.
            throw std::invalid_argument(
                "setVoxelGridProperties:: Input properties must, at least, have a meaningful resolution OR meaningful dimensions."
            );
        }
    }
    else
    {   /// If the adjusted resolution MUST BE EQUAL TO THE INPUT...
        if (set_dimensions)
        {   /// If the adjusted dimensions MUST BE SET...
            if (set_grid_size)
            {   /// If the adjusted grid size MUST BE SET...
                /// The grid size IS SET to a default of 100 voxels in each dimension.
                adjusted_properties.grid_size = Vector3ui(100, 100, 100);
            }

            /// The dimensions IS SET based on the INPUT resolution and grid size (either INPUT, as copied to adjusted, or SET) .
            adjusted_properties.dimensions = adjusted_properties.grid_size.cast<double>() * input_properties.resolution;

            /// Resolution IS INPUT, grid size IS INPUT or IS SET, and dimensions ARE ADJUSTED.
            /// We can now exit function to return valid adjusted_properties.
        }
        else if (!set_dimensions)
        {   /// If the adjusted dimension MUST BE SIMILAR TO THE INPUT...
            /// The grid size IS SET, regardless of the value of `set_grid_size`.
            adjusted_properties.grid_size = (input_properties.dimensions / adjusted_properties.resolution).array().ceil().cast<size_t>();

            /// The dimensions ARE ADJUSTED to match the grid size and dimension.
            adjusted_properties.dimensions = adjusted_properties.grid_size.cast<double>() * input_properties.resolution;

            /// Sanity checks to ensure that dimensions ARE SIMILAR TO THE INPUT.
            /// We want to ensure that we have only expanded the and dimensions.
            assert((adjusted_properties.dimensions.array() >= input_properties.dimensions.array()).all() &&
                    "Adjustments should never shrink the input grid dimensions in any direction.");

            /// Resolution IS INPUT, grid size IS SET or IS ADJUSTED, and dimensions ARE SET.
            /// We can now exit function to return valid adjusted_properties.
        }
    }

    /// The maximum and minimum distances MUST BE positive and negative, respectively. 
    if(adjusted_properties.max_dist < 0) adjusted_properties.max_dist *= 1;
    if(adjusted_properties.min_dist > 0) adjusted_properties.max_dist *= 1;

    return adjusted_properties;
}

VoxelGrid::VoxelGrid(const VoxelGridProperties& properties) :
    ForgeScanEntity(),
    properties(setVoxelGridProperties(properties))
{ 
    setup();
}

VoxelGrid::VoxelGrid(const VoxelGridProperties& properties, const extrinsic& extr) :
    ForgeScanEntity(extr),
    properties(setVoxelGridProperties(properties))
{
    setup();
}

VoxelGrid::VoxelGrid(const VoxelGridProperties& properties, const translation& position) :
    ForgeScanEntity(position),
    properties(setVoxelGridProperties(properties))
{
    setup();
}

VoxelGrid::VoxelGrid(const VoxelGridProperties& properties, const rotation& orientation) :
    ForgeScanEntity(orientation),
    properties(setVoxelGridProperties(properties))
{
    setup();
}

void VoxelGrid::clear()
{
    for (auto& element : voxel_element_vector)
        element.reset();
}

void VoxelGrid::saveHDF5(const std::string& fname) const
{
    try
    {
        HighFive::File file(fname + ".h5", HighFive::File::Truncate);

        auto VoxelElementType = create_compound_VoxelElement();
        VoxelElementType.commit(file, "VoxelElement");

        auto g1 = file.createGroup("VoxelGrid");

        g1.createDataSet("VoxelData", voxel_element_vector);

        g1.createAttribute("Resolution", properties.resolution);
        g1.createAttribute("Dimensions", properties.dimensions);
        g1.createAttribute("GridSize", properties.grid_size);
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
        dset.read(voxel_element_vector);

        /// Cannot use this function right now - My design choice of public but const properties means we cannot
        /// set these variables when we load an HDF5 file.
        /// Instead we could return a new object. Though this would be best as a stand-alone function.
        throw std::logic_error("VoxelGrid::loadHDF5:: Function not implemented.");
        /*
        auto res = g1.getAttribute("Resolution");
        res.read(properties.resolution);

        auto dimensions = g1.getAttribute("Dimensions");
        dimensions.read(properties.dimensions);

        auto grid_size = g1.getAttribute("GridSize");
        grid_size.read(properties.grid_size);
        */
    }
    catch (const HighFive::Exception& err)
    {
        std::cerr << err.what() << std::endl;
    }

}

void VoxelGrid::saveXDMF(const std::string &fname) const
{
    const size_t num_element = voxel_element_vector.size();
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

        for (const auto& voxel_ref : voxel_element_vector)
        {
            vec_updates.push_back(voxel_ref.updates);
            vec_views.push_back(voxel_ref.views);
            vec_min.push_back(voxel_ref.min);
            vec_avg.push_back(voxel_ref.avg);
            vec_var.push_back(voxel_ref.var);
            vec_cent.push_back(voxel_ref.cent);
            vec_norm.push_back(voxel_ref.norm);
            vec_rho.push_back(voxel_ref.rho);
        }

        g1.createDataSet("Updates",    vec_updates);
        g1.createDataSet("Views",      vec_views);
        g1.createDataSet("Minimum",    vec_min);
        g1.createDataSet("Average",    vec_avg);
        g1.createDataSet("Variance",   vec_var);
        g1.createDataSet("Centrality", vec_cent);
        g1.createDataSet("Normality",  vec_norm);
        g1.createDataSet("Density",    vec_rho);

        g1.createAttribute("Resolution", properties.resolution);
        g1.createAttribute("Dimensions", properties.dimensions);
        g1.createAttribute("GridSize", properties.grid_size);

        writeXDMF(fname);
    }
    catch (const HighFive::Exception& err) {
        std::cerr << err.what() << std::endl;
    } 
}

void VoxelGrid::writeXDMF(const std::string &fname) const
{
    std::string hdf5_fname = fname + ".h5";

    const size_t num_element = voxel_element_vector.size();

    point lower_zyx = Eigen::Vector3d::Zero();

    Vector3ui size_1 = properties.grid_size.array() + 1;
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
        "      <DataItem Format=\"XML\" Dimensions=\"3\">" << 
                   properties.resolution << " " << properties.resolution << " " << properties.resolution << "</DataItem>\n"
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

void VoxelGrid::setup()
{
    voxel_element_vector.resize(properties.grid_size.prod());
    double mem = ForgeScan::Utils::byte_to_megabytes(ForgeScan::Utils::vector_capacity(voxel_element_vector));
    if (mem > 100.0)
        std::cout << "Warning, allocated " << mem << " MB for vector grid!" << std::endl;
}


} // ForgeScan

