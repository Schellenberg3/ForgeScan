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
    class Scene;

} // namespace simulation
} // namespace forge_scan


namespace forge_scan {
namespace metrics {
namespace ground_truth {



/// @brief Stores a ground truth for the voxel TSDF of a Scene.
class TSDF : public Grid
{
    /// @brief Required to call the compare method.
    /// friend class metrics::OccupancyConfusion;

    /// @brief Required to call the save method and modify values in the Occupancy Grid.
    friend class simulation::Scene;

public:
    /// @brief Creates a shared pointer to a Ground Truth TSDF Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @return Shared pointer to a TSDF Grid.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties)
    {
        return std::shared_ptr<TSDF>(new TSDF(properties));
    }


    /// @brief Creates a shared pointer to a Ground Truth TSDF Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @return Shared pointer to a TSDF Grid.
    /// @throws `std::runtime_error` If the data vector length does not match the Grid Properties
    ///         number of voxels.
    /// @note   This uses `swap` to exchange the contents of the provided vector with the contents of
    ///         the internal data without copying data.
    static std::shared_ptr<TSDF> create(const std::shared_ptr<const Grid::Properties>& properties,
                                        std::vector<double>& data)
    {
        return std::shared_ptr<TSDF>(new TSDF(properties, data));
    }


    /// @brief Returns the class type name for the Voxel Grid.
    const std::string& getTypeName() const
    {
        static const std::string name = "TSDF";
        return name;
    }


protected:

    /// @brief Compares the ground truth to another vector
    /// @param experiment Vector of experimentally collected data to compare.
    /// @return True of the vectors were the same size and the comparison was performed.
    bool compare(const std::vector<double>& other) const
    {
        const size_t n = this->data.size();
        if (other.size() != n)
        {
            return false;
        }

        for (size_t i = 0; i < n; ++i)
        {
            /// TODO: Decide how to compare.
            double measurement = other[i];
            double truth  = this->data[i];
        }
        return true;
    }


    //// @brief Compares the ground truth to another vector
    /// @param experiment Vector of experimentally collected data to compare.
    /// @return True of the vectors were the same size and the comparison was performed.
    bool compare(const std::vector<float>& other) const
    {
        const size_t n = this->data.size();
        if (other.size() != n)
        {
            return false;
        }

        for (size_t i = 0; i < n; ++i)
        {
            /// TODO: Decide how to compare.
            double measurement = other[i];
            double truth  = this->data[i];

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
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    explicit TSDF(const std::shared_ptr<const Grid::Properties>& properties)
        : Grid(properties,
               DataType::DOUBLE)
    {
        this->data.resize(this->properties->getNumVoxels(), NEGATIVE_INFINITY);
    }


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param properties Shared, constant Grid properties.
    /// @throws `std::runtime_error` If the data vector length does not match the Grid Properties
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
            throw std::runtime_error("Cannot create TSDF. Grid properties and provided data are not the same size.");
        }
        this->data.swap(data);
    } 
};


} // namespace ground_truth
} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_GROUND_TRUTH_TSDF_HPP
