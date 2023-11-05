#ifndef FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_HPP
#define FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_HPP

#include <functional>

#include "ForgeScan/Data/VoxelGrids/VoxelGrid.hpp"


namespace forge_scan {
namespace data {


/// @brief Tracks binary occupancy values for each voxel. The whole VoxelGrid begins as "occupied"
///        and are updated to be "free" rays travel through them.
class Binary : public VoxelGrid
{
public:
    /// @brief Constructor for a shared pointer to an Binary VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param parser ArgParser with arguments to construct an Binary Grid from.
    /// @return Shared pointer to a Binary Grid.
    static std::shared_ptr<Binary> create(const std::shared_ptr<const Grid::Properties>& properties,
                                             const utilities::ArgParser& parser)
    {
        return create(properties, parser.get<float>(VoxelGrid::parse_d_min, VoxelGrid::default_zero),
                                  parser.get<float>(VoxelGrid::parse_d_max, VoxelGrid::default_infinity),
                                  parser.has(Binary::parse_no_occplane));
    }


    /// @brief Constructor for a shared pointer to an Binary VoxelGrid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min    Minimum update distance. Default 0.
    /// @param dist_max    Maximum update distance. Default infinity.
    /// @param no_occplane If true will skips calculating occplane voxel after each update.
    /// @return Shared pointer to an Binary Grid.
    static std::shared_ptr<Binary> create(const std::shared_ptr<const Grid::Properties>& properties,
                                          const float& dist_min = 0,
                                          const float& dist_max = INFINITY,
                                          const bool& no_occplane = false)
    {
        return std::shared_ptr<Binary>(new Binary(properties, dist_min, dist_max, no_occplane));
    }


    /// @return Help message for constructing a Binary VoxelGrid with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: Binary help message";
    }

    /// @brief Returns the class type name for the VoxelGrid.
    const std::string& getTypeName() const override final
    {
        static const std::string name = "Binary";
        return name;
    }

    /// @brief Accessor for `metrics::ground_truth::ExperimentOccupancy` in
    ///       `metrics::OccupancyConfusion`
    /// @return Read-only reference to the Occupancy data vector.
    const std::vector<uint8_t>& getOccupancyData() const
    {
        return std::get<std::vector<uint8_t>>(this->data);
    }


    /// @brief Updates the grid to mark specific voxels as Occplanes.
    void updateOccplanes()
    {
        std::visit(this->update_callable_occplane, this->data);
    }


    /// @brief Updates the grid to mark specific voxels as Occplanes.
    /// @param occplane_centers Storage location for the occplane centers. Any existing data is cleared.
    /// @param occplane_normals Storage location for the occplane normals. Any existing data is cleared.
    void updateOccplanes(std::vector<Eigen::Vector3d>& occplane_centers,
                         std::vector<Eigen::Vector3d>& occplane_normals)
    {
        this->update_callable_occplane.acquireTracking(occplane_centers, occplane_normals);
        std::visit(this->update_callable_occplane, this->data);
        this->update_callable_occplane.releaseTracking();
    }


    /// @brief Updates the Grid with new information along a ray.
    /// @param ray_trace Trace with update voxel location and distances.
    void update(const std::shared_ptr<const Trace>& ray_trace) override final
    {
        this->update_callable.acquireRayTrace(ray_trace);
        std::visit(this->update_callable, this->data);
        this->update_callable.releaseRayTrace();
    }


    void postUpdate() override final
    {
        if (this->no_occplane == false)
        {
            this->updateOccplanes();
        }
    }


    static const std::string type_name;

    static const std::string parse_no_occplane;

private:
    /// @brief Private constructor to enforce shared pointer usage.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param dist_min   Minimum trace update distance for this VoxelGrid.
    /// @param dist_max   Maximum trace update distance for this VoxelGrid.
    /// @param no_occplane If true will skips calculating occplane voxel after each update.
    /// @throws DataVariantError if the DataType is not supported by this VoxelGrid.
    explicit Binary(const std::shared_ptr<const Grid::Properties>& properties,
                    const float& dist_min,
                    const float& dist_max,
                    const bool& no_occplane)
        : VoxelGrid(properties,
                    dist_min,
                    dist_max,
                    VoxelOccupancy::UNSEEN,
                    DataType::UINT8_T,
                    DataType::UINT8_T),
          no_occplane(no_occplane),
          update_callable_occplane(*this),
          update_callable(*this)
    {

    }


    /// @brief Subclass provides update functions for each supported DataType/VectorVariant of
    ///        the data vector.
    struct UpdateCallable : public VoxelGrid::UpdateCallable
    {
        using VoxelGrid::UpdateCallable::operator();

        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<uint8_t>& vector)
        {
            Trace::const_iterator iter = this->ray_trace->first_above(this->caller.dist_min);
            const Trace::const_iterator last_occ  = this->ray_trace->first_above(0, iter);
            const Trace::const_iterator last_free = this->ray_trace->first_above(this->caller.dist_max, last_occ);

            // **************************** APPLY VOXEL UPDATE HERE **************************** //
            for ( ; iter != last_occ; ++iter)
            {
                if (vector[iter->i] != VoxelOccupancy::OCCUPIED)
                {
                    vector[iter->i] = VoxelOccupancy::OCCLUDED;
                }
            }
            for ( ; iter != last_free; ++iter)
            {
                // if (vector[iter->i] != VoxelOccupancy::OCCUPIED)
                {
                    vector[iter->i] = VoxelOccupancy::FREE;
                }
            }
            if (this->ray_trace->hasSensed())
            {
                vector[caller.properties->at(this->ray_trace->sensedPoint())] = VoxelOccupancy::OCCUPIED;
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallable to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallable(Binary& caller)
            : caller(caller)
        {

        }


        /// @brief Reference to the specific derived class calling this object.
        Binary& caller;
    };


    /// @brief Subclass provides occplane calculation functions for each supported DataType/VectorVariant of
    ///        the data vector.
    struct UpdateCallableOccplane : public VoxelGrid::UpdateCallable
    {
        using VoxelGrid::UpdateCallable::operator();

        using implement_function = std::function<void(uint8_t&, const uint8_t&, const uint8_t&, const uint8_t&,
                                                                const uint8_t&, const uint8_t&, const uint8_t&,
                                                                const size_t&,  const size_t&,  const size_t&)>;

        // ************************************************************************************* //
        // *                                SUPPORTED DATATYPES                                * //
        // ************************************************************************************* //


        void operator()(std::vector<uint8_t>& vector)
        {
            static const GridSize minGridSize = GridSize(3, 3, 3);
            if ((this->caller.properties->size.array() < minGridSize.array()).any())
            {
                return;
            }

            implement_function implement;
            if (this->occplane_centers && this->occplane_normals)
            {
                this->occplane_centers->clear();
                this->occplane_normals->clear();
                implement = this->implement_track;
            }
            else
            {
                implement = this->implement_no_track;
            }

            const size_t dx = 1;
            const size_t dy = this->caller.properties->size.x();
            const size_t dz = this->caller.properties->size.x() * this->caller.properties->size.y();

            for (size_t z = 1; z < this->caller.properties->size.z() - 1; ++z)
            {
                for (size_t y = 1; y < this->caller.properties->size.y() - 1; ++y)
                {
                    for (size_t x = 1; x < this->caller.properties->size.x() - 1; ++x)
                    {
                        size_t c_idx = this->caller.properties->operator[](Index(x, y, z));
                        uint8_t& c  = vector[c_idx];
                        uint8_t& px = vector[c_idx + dx];
                        uint8_t& nx = vector[c_idx - dx];
                        uint8_t& py = vector[c_idx + dy];
                        uint8_t& ny = vector[c_idx - dy];
                        uint8_t& pz = vector[c_idx + dz];
                        uint8_t& nz = vector[c_idx - dz];

                        if (c & VoxelOccupancy::TYPE_UNKNOWN)
                        {
                            implement(c, px, nx, py, ny, pz, nz, x, y, z);
                        }
                    }
                }
            }
        }


        // ************************************************************************************* //
        // *                         PUBLIC CLASS METHODS AND MEMBERS                          * //
        // ************************************************************************************* //


        /// @brief Creates an UpdateCallableOccplane to implement the derived class's update function.
        /// @param caller Reference to the specific derived class calling this object.
        UpdateCallableOccplane(Binary& caller)
            : caller(caller)
        {
            using namespace std::placeholders;
            this->implement_track    = std::bind(&UpdateCallableOccplane::implementTrack,   this, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10);
            this->implement_no_track = std::bind(&UpdateCallableOccplane::implementNoTrack, this, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10);
        }


        /// @brief Acquires the centers and normals vectors in this class' member variables.
        /// @param occplane_centers Storage location for occplane centers.
        /// @param occplane_normals Storage location for occplane normals.
        void acquireTracking(std::vector<Eigen::Vector3d>& occplane_centers,
                             std::vector<Eigen::Vector3d>& occplane_normals)
        {
            this->occplane_centers = &occplane_centers;
            this->occplane_normals = &occplane_normals;
        }


        /// @brief Resets the class member variables pointing to the storage location of occplane centers
        ///        and normals to nullptr, releasing temporary ownership.
        void releaseTracking()
        {
            this->occplane_centers = nullptr;
            this->occplane_normals = nullptr;
        }


        /// @brief Implementation that only sets the voxel label to an occplane type.
        void implementNoTrack(uint8_t& c,
                              const uint8_t& px, const uint8_t& nx,
                              const uint8_t& py, const uint8_t& ny,
                              const uint8_t& pz, const uint8_t& nz,
                              const size_t&, const size_t&, const size_t&)
        {
            if (px & VoxelOccupancy::TYPE_FREE || nx & VoxelOccupancy::TYPE_FREE ||
                py & VoxelOccupancy::TYPE_FREE || ny & VoxelOccupancy::TYPE_FREE ||
                pz & VoxelOccupancy::TYPE_FREE || nz & VoxelOccupancy::TYPE_FREE)
            {
                c |= VoxelOccupancy::TYPE_OCCPLANE;
            }
        }


        /// @brief That sets the voxel label to an occplane type and records the location and
        ///        normal for each occplane.
        void implementTrack(uint8_t& c,
                            const uint8_t& px, const uint8_t& nx,
                            const uint8_t& py, const uint8_t& ny,
                            const uint8_t& pz, const uint8_t& nz,
                            const size_t& x, const size_t& y, const size_t& z)
        {
            if (c & VoxelOccupancy::TYPE_UNKNOWN)
            {
                Eigen::Vector3d normal = Eigen::Vector3d::Zero();

                normal.x() += ((px & VoxelOccupancy::TYPE_FREE) == VoxelOccupancy::TYPE_FREE) * 1;
                normal.x() -= ((nx & VoxelOccupancy::TYPE_FREE) == VoxelOccupancy::TYPE_FREE) * 1;
                normal.y() += ((py & VoxelOccupancy::TYPE_FREE) == VoxelOccupancy::TYPE_FREE) * 1;
                normal.y() -= ((ny & VoxelOccupancy::TYPE_FREE) == VoxelOccupancy::TYPE_FREE) * 1;
                normal.z() += ((pz & VoxelOccupancy::TYPE_FREE) == VoxelOccupancy::TYPE_FREE) * 1;
                normal.z() -= ((nz & VoxelOccupancy::TYPE_FREE) == VoxelOccupancy::TYPE_FREE) * 1;

                if (normal.isZero(0.0f) == false)
                {
                    c |= VoxelOccupancy::TYPE_OCCPLANE;

                    const float& res = this->caller.properties->resolution;
                    this->occplane_centers->emplace_back(x * res, y * res, z * res);
                    this->occplane_normals->emplace_back(normal.normalized());
                }
            }
        }


        /// @brief Reference to the specific derived class calling this object.
        Binary& caller;

        /// @brief Function pointers to the implementation callbacks for the two variations of the occplane update.
        implement_function implement_track, implement_no_track;

        /// @brief Locations for the `implementTrack` method to store occplane centers and normals in.
        std::vector<Eigen::Vector3d> *occplane_centers = nullptr, *occplane_normals = nullptr;
    };


    /// @brief If true will skip calculating the occplanes after each update.
    bool no_occplane;

    /// @brief Subclass callable that std::visit uses to perform occplane calculation with typed information.
    UpdateCallableOccplane update_callable_occplane;

    /// @brief Subclass callable that std::visit uses to perform updates with typed information.
    /// @note  Initialization order matters. This musts be declared last so the other class members that
    ///        this uses are guaranteed to be initialized.
    UpdateCallable update_callable;
};


/// @brief String for the class name.
const std::string Binary::type_name = "Binary";

/// @brief Parser flag to skip calculating the occplanes.
const std::string Binary::parse_no_occplane = "--no-occplane";


} // namespace data
} // namespace forge_scan


#endif // FORGE_SCAN_RECONSTRUCTIONS_GRID_BINARY_HPP
