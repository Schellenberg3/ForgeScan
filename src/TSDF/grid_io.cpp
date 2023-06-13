#include <iostream>
#include <fstream>
#include <iomanip>

/// Eigen is used when saving/loading from HDF5 files.
#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>

#include "ForgeScan/forgescan_types.h"
#include "ForgeScan/TSDF/grid.h"


/// @brief Helper for Grid::saveHDF5 to inform HighFive of the datatypes represented in the `voxel_vector`.
static HighFive::CompoundType create_compound_Voxel() {
    return {
        {"views",    HighFive::AtomicType<ForgeScan::view_count>{}},
        {"updates",  HighFive::AtomicType<ForgeScan::update_count>{}},

        {"min",  HighFive::AtomicType<ForgeScan::voxel_dist>{}},
        {"avg",  HighFive::AtomicType<ForgeScan::voxel_dist>{}},
        {"var",  HighFive::AtomicType<ForgeScan::voxel_dist>{}},

        {"cent", HighFive::AtomicType<ForgeScan::centrality>{}},
        {"norm", HighFive::AtomicType<ForgeScan::normality>{}},
        {"rho", HighFive::AtomicType<ForgeScan::density>{}}
    };
}
HIGHFIVE_REGISTER_TYPE(ForgeScan::TSDF::Voxel, create_compound_Voxel)


namespace ForgeScan {
namespace TSDF {


void Grid::saveHDF5(const std::filesystem::path& fname) const
{
    if ( !fname.has_filename() )
        throw std::invalid_argument("[Grid::saveHDF5] Invalid file name! Could not identify filename.");
    try
    {
        HighFive::File file(fname.string() + ".h5", HighFive::File::Truncate);

        auto VoxelType = create_compound_Voxel();
        VoxelType.commit(file, "Voxel");

        auto g1 = file.createGroup("Grid");

        g1.createDataSet("VoxelData", voxel_vector);

        g1.createAttribute("MinDist",    properties.min_dist);
        g1.createAttribute("MaxDist",    properties.max_dist);
        g1.createAttribute("Resolution", properties.resolution);
        g1.createAttribute("Dimensions", properties.dimensions);
        g1.createAttribute("GridSize",   properties.grid_size);
        g1.createAttribute("Extrinsic",  extr.matrix());
    }
    catch (const HighFive::Exception& err)
    {
        std::cerr << err.what() << std::endl;
    }
}

void Grid::saveXDMF(const std::filesystem::path &fname) const
{
    if ( !fname.has_filename() )
        throw std::invalid_argument("[Grid::saveXDMF] Invalid file name! Could not identify filename.");
    const size_t num_voxels = voxel_vector.size();
    try
    {
        HighFive::File file(fname.string() + ".h5", HighFive::File::Truncate);

        auto VoxelType = create_compound_Voxel();
        VoxelType.commit(file, "Voxel");

        auto g1 = file.createGroup("Grid");

        std::vector<update_count> vec_updates;
        std::vector<view_count>   vec_views;
        std::vector<voxel_dist>   vec_min;
        std::vector<voxel_dist>   vec_avg;
        std::vector<voxel_dist>   vec_var;
        std::vector<centrality>   vec_cent;
        std::vector<normality>    vec_norm;
        std::vector<density>      vec_rho;

        vec_updates.reserve(num_voxels);
        vec_views.reserve(num_voxels);
        vec_min.reserve(num_voxels);
        vec_avg.reserve(num_voxels);
        vec_var.reserve(num_voxels);
        vec_cent.reserve(num_voxels);
        vec_norm.reserve(num_voxels);
        vec_rho.reserve(num_voxels);

        for (const auto& voxel_ref : voxel_vector)
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

        g1.createAttribute("MinDist",    properties.min_dist);
        g1.createAttribute("MaxDist",    properties.max_dist);
        g1.createAttribute("Resolution", properties.resolution);
        g1.createAttribute("Dimensions", properties.dimensions);
        g1.createAttribute("GridSize",   properties.grid_size);
        g1.createAttribute("Extrinsic",  extr.matrix());
        writeXDMF(fname);
    }
    catch (const HighFive::Exception& err) {
        std::cerr << err.what() << std::endl;
    }
}


void Grid::writeXDMF(const std::filesystem::path &fname) const
{
    std::string hdf5_fname = fname.filename().string() + ".h5";

    const size_t num_voxels = voxel_vector.size();

    point lower_zyx = Eigen::Vector3d::Zero();
    lower_zyx.array() -= 0.5 * properties.resolution;

    Vector3ui adj_size = properties.grid_size.array() + 1;
    adj_size.reverseInPlace();

    std::ofstream file;
    file.exceptions( std::ofstream::failbit | std::ofstream::badbit );
    try
    {
        file.open(fname.string() + ".xdmf");
        file << std::fixed << std::setprecision(8);
        file <<
        "<?xml version=\"1.0\"?>\n"
        "<!DOCTYPE Xdmf SYSTEM \"Xdmf.dtd\"[]>\n"
        "<Xdmf xmlns:xi=\"http://www.w3.org/2003/XInclude\" Version=\"2.2\">\n"
        " <Domain>\n"
        "  <!-- *************** START OF GRID *************** -->\n"
        "  <Grid Name=\"GRID\" GridType=\"Uniform\">\n"
        "    <Geometry Type=\"ORIGIN_DXDYDZ\">\n"
        "      <Topology TopologyType=\"3DCoRectMesh\" Dimensions=\"" << adj_size.transpose() << " \"></Topology>\n" <<
        "      <!-- Origin  Z, Y, X -->\n"
        "      <DataItem Format=\"XML\" Dimensions=\"3\">" << lower_zyx.transpose() << "</DataItem>\n" <<
        "      <!-- DxDyDz (Spacing/Resolution) Z, Y, X -->\n"
        "      <DataItem Format=\"XML\" Dimensions=\"3\">" <<
                   properties.resolution << " " << properties.resolution << " " << properties.resolution << "</DataItem>\n"
        "    </Geometry>\n"

        //  Updates
        "    <Attribute Name=\"Updates\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Int\" Precision=\"2\" >\n"
        "        " << hdf5_fname << ":/Grid/Updates\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Views
        "    <Attribute Name=\"Views\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Int\" Precision=\"2\" >\n"
        "        " << hdf5_fname << ":/Grid/Views\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Minimum
        "    <Attribute Name=\"Minimum\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/Grid/Minimum\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Average
        "    <Attribute Name=\"Average\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/Grid/Average\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Variance
        "    <Attribute Name=\"Variance\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/Grid/Variance\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Centrality
        "    <Attribute Name=\"Centrality\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/Grid/Centrality\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Normality
        "    <Attribute Name=\"Normality\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/Grid/Normality\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        //  Density
        "    <Attribute Name=\"Density\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
        "      <DataItem Format=\"HDF\" Dimensions=\"" << num_voxels << "\" NumberType=\"Float\" Precision=\"8\" >\n"
        "        " << hdf5_fname << ":/Grid/Density\n"
        "      </DataItem>\n"
        "    </Attribute>\n"

        " </Grid>\n"
        " <!-- *************** END OF GRID *************** -->\n"
        " </Domain>\n"
        "</Xdmf>\n"
        << std::endl;
    }
    catch (std::ofstream::failure e)
    {
        std::cerr << "Encountered error in Grid::writeXDMF. See message:\n" << e.what();
    }
}

Grid loadGridHDF5(const std::filesystem::path& fname)
{
    if ( !fname.has_filename() )
        throw std::invalid_argument("[Grid::loadHDF5] Invalid file name! Could not identify filename.");
    try
    {
        HighFive::File file(fname.string() + ".h5", HighFive::File::ReadOnly);
        Grid::Properties properties;
        auto g1 = file.getGroup("Grid");

        auto res = g1.getAttribute("Resolution");
        res.read(properties.resolution);

        auto dimensions = g1.getAttribute("Dimensions");
        dimensions.read(properties.dimensions);

        auto grid_size = g1.getAttribute("GridSize");
        grid_size.read(properties.grid_size);

        auto min_dist = g1.getAttribute("MinDist");
        min_dist.read(properties.min_dist);

        auto max_dist = g1.getAttribute("MaxDist");
        max_dist.read(properties.max_dist);

        extrinsic new_extr;
        auto extr = g1.getAttribute("Extrinsic");
        extr.read(new_extr.matrix());

        Grid new_grid(properties);

        auto dset = g1.getDataSet("VoxelData");
        dset.read(new_grid.voxel_vector);

        new_grid.transformWorldFrame(new_extr);

        return new_grid;
    }
    catch (const HighFive::Exception& err)
    {
        std::cerr << err.what() << std::endl;
        throw err;
    }
}


} // TSDF
} // ForgeScan
