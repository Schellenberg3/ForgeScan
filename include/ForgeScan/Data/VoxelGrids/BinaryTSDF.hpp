#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_TSDF_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_TSDF_HPP

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"
#include "ForgeScan/Utilities/Math.hpp"


namespace forge_scan {
namespace data {


/// @brief Represents a truncated signed-distance function and binary occupation.
/// @note Supports `float` and `double` data types only. These are for the TSDF. Binary occupation
///       is always `uint8_t`.
class BinaryTSDF : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to a Binary TSDF VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param parser ArgParser with arguments to construct an Binary TSDF Grid from.
    /// @return Shared pointer to a Binary TSDF Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<BinaryTSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                                 const utilities::ArgParser& parser)
    {
        return create(properties, parser.get<float>(VoxelGrid::parse_d_min, VoxelGrid::default_d_min),
                                  parser.get<float>(VoxelGrid::parse_d_max, VoxelGrid::default_d_max),
                                  stringToDataType(parser.get(VoxelGrid::parse_dtype), DataType::FLOAT));
    }


    /// @brief Constructor for a shared pointer to a Binary TSDF VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min   Minimum update distance. Default -0.2.
    /// @param dist_max   Maximum update distance. Default +0.2.
    /// @param type_id Datatype for the Grid. Default is float.
    /// @return Shared pointer to a Binary TSDF Grid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    static std::shared_ptr<BinaryTSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                                 const float& dist_min = -0.2,
                                                 const float& dist_max =  0.2,
                                                 const DataType& type_id = DataType::FLOAT)
    {
        return std::shared_ptr<BinaryTSDF>(new BinaryTSDF(properties, dist_min, dist_max, type_id));
    }


    /// @return Help message for constructing a Binary TSDF VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: BinaryTSDF help message";
    }


    /// @brief Returns the class type name for the VoxelGrid.
    const std::string& getTypeName() const override final
    {
        return BinaryTSDF::type_name;
    }


    /// @brief Accessor for `metrics::ground_truth::ExperimentOccupancy` in
    ///       `metrics::OccupancyConfusion`
    /// @return Read-only reference to the Occupancy data vector.
    const std::vector<uint8_t>& getOccupancyData() const
    {
        return this->data_occupancy;
    }


    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(const std::shared_ptr<const Trace>& ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }

    static const std::string type_name;

private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min   Minimum trace update distance for this VoxelGrid.
    /// @param dist_max   Maximum trace update distance for this VoxelGrid.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    explicit BinaryTSDF(const std::shared_ptr<const Grid::Properties>& properties,
                           const float& dist_min,
                           const float& dist_max,
                           const DataType& type_id)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    NEGATIVE_INFINITY,
                    type_id,
                    DataType::TYPE_FLOATING_POINT),
          data_occupancy(std::vector<uint8_t>(this->properties->getNumVoxels(),
                                              VoxelOccupancy::UNSEEN)),
          update_callable(*this)
    {

    }


    void save(HighFive::Group& g_channel, const std::string& grid_type) override final
    {
        try
        {
            if (this->type_id == DataType::FLOAT)
            {
                g_channel.createDataSet(grid_type + "_tsdf", std::get<std::vector<float>>(this->data));
            }
            else if (this->type_id == DataType::DOUBLE)
            {
                g_channel.createDataSet(grid_type + "_tsdf", std::get<std::vector<double>>(this->data));
            }
            else
            {
                throw DataVariantError::UnrecognizedEnumeration(this->type_id);
            }
        }
        catch (const std::bad_variant_access&)
        {
            throw std::runtime_error("Bad variant access: Cannot save Grid's Data vector to HDF5 file.");
        }

        g_channel.createDataSet(grid_type + "_binary", this->data_occupancy);
    }


    void addToXDMF(std::ofstream& file,          const std::string& hdf5_fname,
                   const std::string& grid_name, const std::string& grid_type) const override final
    {
        utilities::XDMF::writeVoxelGridAttribute(
            file,
            grid_name + "_tsdf",
            utilities::XDMF::makeDataPath(hdf5_fname, FS_HDF5_RECONSTRUCTION_GROUP, grid_name, grid_type + "_tsdf"),
            getNumberTypeXDMF(this->type_id),
            getNumberPrecisionXDMF(this->type_id),
            this->properties->getNumVoxels()
        );

        utilities::XDMF::writeVoxelGridAttribute(
            file,
            grid_name + "_binary",
            utilities::XDMF::makeDataPath(hdf5_fname, FS_HDF5_RECONSTRUCTION_GROUP, grid_name, grid_type + "_binary"),
            getNumberTypeXDMF(DataType::UINT8_T),
            getNumberPrecisionXDMF(DataType::UINT8_T),
            this->properties->getNumVoxels()
        );
    }


    /// @brief Subclass provides update functions for each supported DataType/VectorVariant of
    ///        the data vector.
    struct UpdateCallable : public VoxelGrid::UpdateCallable
    {
        using VoxelGrid::UpdateCallable::operator();

        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<float>& vector)
        {
            Trace::const_iterator iter            = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last_occ  = this->ray_trace->first_above(0, iter);
            const Trace::const_iterator last_free = this->ray_trace->first_above(this->caller.dist_max, last_occ);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last_occ; ++iter)
            {
                this->caller.data_occupancy[iter->i] = VoxelOccupancy::OCCUPIED;
                vector[iter->i] = utilities::math::smallest_magnitude(vector[iter->i], iter->d);
            }
            for ( ; iter != last_free; ++iter)
            {
                this->caller.data_occupancy[iter->i] = VoxelOccupancy::FREE;
                vector[iter->i] = utilities::math::smallest_magnitude(vector[iter->i], iter->d);
            }
        }


        void operator()(std::vector<double>& vector)
        {
            Trace::const_iterator iter            = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last_occ  = this->ray_trace->first_above(0, iter);
            const Trace::const_iterator last_free = this->ray_trace->first_above(this->caller.dist_max, last_occ);
            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last_occ; ++iter)
            {
                this->caller.data_occupancy[iter->i] = VoxelOccupancy::OCCUPIED;
                vector[iter->i] = utilities::math::smallest_magnitude(vector[iter->i], static_cast<double>(iter->d));
            }
            for ( ; iter != last_free; ++iter)
            {
                this->caller.data_occupancy[iter->i] = VoxelOccupancy::FREE;
                vector[iter->i] = utilities::math::smallest_magnitude(vector[iter->i], static_cast<double>(iter->d));
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(BinaryTSDF& caller)
            : caller(caller)
        {

        }

        /// @brief Reference to the specific derived class calling this object.
        BinaryTSDF& caller;
    };


    /// @brief Stores the occupancy data that the grid uses.
    std::vector<uint8_t> data_occupancy;

    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This must be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};


/// @brief String for the class name.
const std::string BinaryTSDF::type_name = "BinaryTSDF";



} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_TSDF_HPP
