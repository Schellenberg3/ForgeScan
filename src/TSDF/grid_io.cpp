#include <iostream>
#include <fstream>
#include <iomanip>

/// Eigen is used when saving/loading from HDF5 files.
#define H5_USE_EIGEN 1
#include <highfive/H5File.hpp>

#include "ForgeScan/TSDF/grid.h"


namespace ForgeScan {
namespace TSDF      {


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

        Grid new_grid(properties);

        extrinsic new_extr;
        auto extr = g1.getAttribute("Extrinsic");
        extr.read(new_extr.matrix());
        new_grid.transformWorldFrame(new_extr);

        auto updates = g1.getDataSet("Updates");
        updates.read(new_grid.v_update_count);

        auto views = g1.getDataSet("Views");
        views.read(new_grid.v_view_count);

        auto min = g1.getDataSet("Minimum");
        min.read(new_grid.v_voxel_dist_min);

        auto avg = g1.getDataSet("Average");
        avg.read(new_grid.v_voxel_dist_avg);

        auto var = g1.getDataSet("Variance");
        var.read(new_grid.v_voxel_dist_var);

        return new_grid;
    }
    catch (const HighFive::Exception& err)
    {
        std::cerr << err.what() << std::endl;
        throw err;
    }
}


void Grid::saveXDMF(const std::filesystem::path &fname) const
{
    if ( !fname.has_filename() )
        throw std::invalid_argument("[Grid::saveXDMF] Invalid file name! Could not identify filename.");
    try
    {
        HighFive::File file(fname.string() + ".h5", HighFive::File::Truncate);

        auto g1 = file.createGroup("Grid");

        g1.createDataSet("Updates",    v_update_count);
        g1.createDataSet("Views",      v_view_count);
        g1.createDataSet("Minimum",    v_voxel_dist_min);
        g1.createDataSet("Average",    v_voxel_dist_avg);
        g1.createDataSet("Variance",   v_voxel_dist_var);

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

    const size_t num_voxels = this->properties.grid_size.prod();

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



} // namespace TSDF
} // namespace ForgeScan
