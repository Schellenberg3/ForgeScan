#ifndef FORGE_SCAN_METRICS_GROUND_TRUTH_TSDF_HPP
#define FORGE_SCAN_METRICS_GROUND_TRUTH_TSDF_HPP

#include "ForgeScan/Common/Grid.hpp"


namespace forge_scan {
namespace metrics {

    // Forward definition to allow friend access.
    /// TODO: Create the Metric which uses this.

} // namespace metrics

namespace simulation {

    // Forward definition to allow friend access.
    struct Scene;
    struct GroundTruthScene;

} // namespace simulation
} // namespace forge_scan


namespace forge_scan {
namespace metrics {
namespace ground_truth {



/// @brief Stores a ground truth for the voxel TSDF of a Scene.
class TSDF : public Grid
{
    /// @details Required to call the compare method.
    /// TODO: Create the Metric which uses this.
    /// friend class metrics::OccupancyConfusion;

    /// @details Required to modify values in the grid.
    friend struct simulation::Scene;

    /// @details Required to call the save method.
    friend struct simulation::GroundTruthScene;

public:
    /// @brief Creates a shared pointer to a Ground Truth TSDF Grid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @return Shared pointer to a TSDF Grid.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties)
    {
        return std::shared_ptr<TSDF>(new TSDF(properties));
    }


    /// @brief Creates a shared pointer to a Ground Truth TSDF Grid.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    /// @param data The data (typically loaded from an HDF5) for the TSDF Grid.
    /// @return Shared pointer to a TSDF Grid.
    /// @throws GridPropertyError If the data vector length does not match the `Grid::Properties`
    ///         number of voxels.
    /// @note   This uses `swap` to exchange the contents of the provided vector with the contents of
    ///         the internal data without copying data.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                        std::vector<double>& data)
    {
        return std::shared_ptr<TSDF>(new TSDF(properties, data));
    }


    /// @brief Returns the class type name for the Grid.
    const std::string& getTypeName() const override final
    {
        return TSDF::type_name;
    }


    static const std::string type_name;

protected:

    /// @brief Compares the ground truth to another vector
    /// @param experiment Vector of experimentally collected data to compare.
    /// @return True of the vectors were the same size and the comparison was performed.
    bool compare(const std::vector<double>& experiment) const
    {
        const size_t n = this->data.size();
        if (experiment.size() != n)
        {
            return false;
        }

        for (size_t i = 0; i < n; ++i)
        {
            /// TODO: Decide how to compare.
            // double measurement = other[i];
            // double truth  = this->data[i];
        }
        return true;
    }


    //// @brief Compares the ground truth to another vector
    /// @param experiment Vector of experimentally collected data to compare.
    /// @return True of the vectors were the same size and the comparison was performed.
    bool compare(const std::vector<float>& experiment) const
    {
        const size_t n = this->data.size();
        if (experiment.size() != n)
        {
            return false;
        }

        for (size_t i = 0; i < n; ++i)
        {
            /// TODO: Decide how to compare.
            // double measurement = other[i];
            // double truth  = this->data[i];

        }
        return true;
    }


    /// @brief Writes the Grid's data vector to the provided HDF5 group.
    /// @param group Group in the opened HDF5 file.
    /// @return DataSet Object.
    HighFive::DataSet save(HighFive::Group& group) const
    {
        return group.createDataSet(this->getTypeName(), this->data);
    }


    double& at(Index idx)
    {
        return this->at(this->properties->at(idx));
    }


    double& at(size_t n)
    {
        return this->data.at(n);
    }


    double& operator[](Index idx)
    {
        return this->at(this->properties->operator[](idx));
    }


    double& operator[](size_t n)
    {
        return this->data[n];
    }


    /// @brief Ground truth data.
    std::vector<double> data;


private:
    /// @brief Private constructor to enforce use of shared pointers.
    /// @param properties Shared, constant pointer to the `Grid::Properties` to use.
    explicit TSDF(const std::shared_ptr<const Grid::Properties>& properties)
        : Grid(properties,
               DataType::DOUBLE)
    {
        this->data.resize(this->properties->getNumVoxels(), NEGATIVE_INFINITY);
    }


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param properties Shared, constant `Grid::Properties`.
    /// @throws GridPropertyError If the data vector length does not match the `Grid::Properties`
    ///         number of voxels.
    /// @note   This uses `swap` to exchange the contents of the provided vector with the contents of
    ///         the internal data without copying data.
    explicit TSDF(const std::shared_ptr<const Grid::Properties>& properties,
                  std::vector<double>& data)
        : Grid(properties,
               DataType::DOUBLE)
    {
        if (this->properties->getNumVoxels() != data.size())
        {
            throw GridPropertyError::DataVectorDoesNotMatch(this->properties->size, data.size());
        }
        this->data.swap(data);
    }
};


/// @brief String for the class name.
const std::string TSDF::type_name = "TSDF";


} // namespace ground_truth
} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_GROUND_TRUTH_TSDF_HPP
