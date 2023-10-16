#ifndef FORGE_SCAN_UTILITIES_XDMF_HPP
#define FORGE_SCAN_UTILITIES_XDMF_HPP

#include <cstddef>
#include <fstream>
#include <iomanip>


namespace forge_scan {
namespace utilities {
namespace XDMF {


/// @brief Writes an XDMF header to the file.
/// @param file An opened file stream to write to.
inline void writeHeader(std::ofstream& file)
{
    file <<
    "<?xml version=\"1.0\"?>\n"
    "<!DOCTYPE Xdmf SYSTEM \"Xdmf.dtd\"[]>\n"
    "<Xdmf xmlns:xi=\"http://www.w3.org/2003/XInclude\" Version=\"2.2\">\n"
    " <Domain>\n";
}


/// @brief Writes The header information for a Grid into the XDMF file.
/// @param file An opened file stream to write to.
/// @param resolution Voxel edge length.
/// @param nx Number of voxel in the X-direction.
/// @param ny Number of voxel in the X-direction.
/// @param nz Number of voxel in the X-direction.
/// @param ox Origin location along the X-axis.
/// @param oy Origin location along the X-axis.
/// @param oz Origin location along the X-axis.
inline void writeVoxelGridHeader(std::ofstream& file, const float& resolution,
                                 const size_t& nx, const size_t& ny, const size_t& nz,
                                 const float& ox, const float& oy, const float& oz)
{
    file <<
    "  <!-- *************** START OF VOXEL GRID *************** -->\n"
    "  <Grid Name=\"VOXEL_GRID\" GridType=\"Uniform\">\n"
    "    <Geometry Type=\"ORIGIN_DXDYDZ\">\n"
    "      <Topology TopologyType=\"3DCoRectMesh\" Dimensions=\"" << nx << " " << ny << " " << nz << " \"></Topology>\n" <<
    "      <!-- Origin  Z, Y, X -->\n"
    "      <DataItem Format=\"XML\" Dimensions=\"3\">" << oz << " " << oy << " " << ox << "</DataItem>\n" <<
    "      <!-- DxDyDz (Spacing/Resolution) Z, Y, X -->\n"
    "      <DataItem Format=\"XML\" Dimensions=\"3\">" << resolution << " " << resolution << " " << resolution << "</DataItem>\n"
    "    </Geometry>\n";
}


/// @brief Constructs an XDMF data path string for the data's location in the HDF5 file.
/// @param hdf5_fname  File name, with extention, for the HDF5 file which stores the attribute information.
/// @param group_name  Parent group of the attribute (e.g., is it a Reconstruction or Metric)
/// @param sub_group_name Sub-group for the data.
/// @param dset_name   Name of the Dataset in the HDF5 file. This is the specific name of the type of data channel.
/// @return XDMF data path string.
inline std::string makeDataPath(const std::string& hdf5_fname,
                                const std::string& group_name,
                                const std::string& sub_group_name,
                                const std::string& dset_name)
{
    return hdf5_fname + ":/" + group_name + "/" + sub_group_name +  "/" + dset_name;
}


/// @brief Constructs an XDMF data path string for the data's location in the HDF5 file.
/// @param hdf5_fname  File name, with extention, for the HDF5 file which stores the attribute information.
/// @param group_name  Parent group of the attribute (e.g., is it a Reconstruction or Metric)
/// @param dset_name   Name of the Dataset in the HDF5 file. This is the specific name of the type of data channel.
/// @return XDMF data path string.
inline std::string makeDataPath(const std::string& hdf5_fname,
                                const std::string& group_name,
                                const std::string& dset_name)
{
    return hdf5_fname + ":/" + group_name + "/"  + dset_name;
}


/// @brief Constructs an XDMF data path string for the data's location in the HDF5 file.
/// @param hdf5_fname  File name, with extention, for the HDF5 file which stores the attribute information.
/// @param dset_path   Full internal HDF5 path to a dataset.
/// @return XDMF data path string.
inline std::string makeDataPath(const std::string& hdf5_fname,
                                const std::string& dset_path)
{
    return hdf5_fname + ":/" + dset_path;
}


/// @brief Writes The attribute information about a Grid into the XDMF file.
/// @param file An opened file stream to write to.
/// @param attr_name   Name of the attribute. This is the user's provided name for the data channel.
/// @param data_path   Path from the HDF5 root to the dataset for the XDMF item.
/// @param number_type Numeric type for the Grid. One of "Char", "UChar", "Int", "UInt", or  "Float".
/// @param precision   Precision for the numeric type. On of "1", "2", "4", or "8". Should match the numeric type.
/// @param n_voxel     Total number of voxels in the Grid.
inline void writeVoxelGridAttribute(std::ofstream& file,
                                    const std::string& attr_name,
                                    const std::string& data_path,
                                    const std::string& number_type,
                                    const std::string& precision,
                                    const size_t& n_voxel)
{
    file <<
    "    <Attribute Name=\"" << attr_name << "\" AttributeType=\"Scalar\" Center=\"Cell\">\n"
    "      <DataItem Format=\"HDF\" Dimensions=\"" << n_voxel  << "\" NumberType=\"" << number_type << "\" Precision=\""  << precision << "\" >\n"
    "        " << data_path << "\n"
    "      </DataItem>\n"
    "    </Attribute>\n";
}


/// @brief Writes The footer information for a Grid into the XDMF file.
/// @param file An opened file stream to write to.
inline void writeVoxelGridFooter(std::ofstream& file)
{
    file <<
    "  </Grid>\n"
    "  <!-- *************** END OF VOXEL GRID *************** -->\n";
}


/// @brief Writes The footer information into the XDMF file.
/// @param file An opened file stream to write to.
inline void writeFooter(std::ofstream& file)
{
    file <<
    " </Domain>\n"
    "</Xdmf>";
}


} // namespace XDMF
} // namespace utilities
} // namespace forge_scan


#endif // FORGE_SCAN_UTILITIES_XDMF_HPP
