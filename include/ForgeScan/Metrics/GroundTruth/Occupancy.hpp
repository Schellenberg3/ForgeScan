#ifndef FORGE_SCAN_METRICS_GROUND_TRUTH_OCCUPANCY_HPP
#define FORGE_SCAN_METRICS_GROUND_TRUTH_OCCUPANCY_HPP

#include <variant>

#include "ForgeScan/Common/Grid.hpp"


namespace forge_scan {
namespace metrics {

    // Forward definition to allow friend access.
    class OccupancyConfusion;

} // namespace metrics

namespace simulation {

    // Forward definition to allow friend access.
    struct Scene;

} // namespace simulation
} // namespace forge_scan


namespace forge_scan {
namespace metrics {
namespace ground_truth {


/// @brief Stores the attributes of a confusion matrix for comparing a Ground Truth
///        Occupancy Grid to an experimentally collected Occupancy VoxelGrid.
struct Confusion
{
    /// @brief Sets all values to zero.
    void reset()
    {
        this->tp = 0;
        this->tn = 0;
        this->fp = 0;
        this->fn = 0;
        this->uk = 0;
    }


    /// @return Sum of all elements in the confusion element.
    size_t sum() const
    {
        return this->tp + this->tn + this->fp + this->fn +this->uk;
    }


    /// @brief The count of true positives between the ground truth and experimental data.
    size_t tp = 0;

    /// @brief The count of true negatives between the ground truth and experimental data.
    size_t tn = 0;

    /// @brief The count of false positives between the ground truth and experimental data.
    size_t fp = 0;

    /// @brief The count of false negatives between the ground truth and experimental data.
    size_t fn = 0;

    /// @brief The count of unknown elements in the experimental data.
    size_t uk = 0;
};


/// @brief Stores a ground truth for the voxel occupancy of a Scene.
class Occupancy : public Grid
{
    /// @brief Required to call the compare method.
    friend class metrics::OccupancyConfusion;

    /// @brief Required to call the save method and modify values in the Occupancy Grid.
    friend struct simulation::Scene;


public:
    /// @brief Creates a shared pointer to a Ground Truth Occupancy Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @return Shared pointer to an Occupancy Grid.
    static std::shared_ptr<Occupancy> create(const std::shared_ptr<const Grid::Properties>& properties)
    {
        return std::shared_ptr<Occupancy>(new Occupancy(properties));
    }


    /// @brief Creates a shared pointer to a Ground Truth Occupancy Grid.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @param data The data (typically loaded from an HDF5) for the Occupancy Grid.
    /// @return Shared pointer to an Occupancy Grid.
    /// @throws GridPropertyError If the data vector length does not match the Grid Properties
    ///         number of voxels.
    /// @note   This uses `swap` to exchange the contents of the provided vector with the contents of
    ///         the internal data without copying data.
    static std::shared_ptr<Occupancy> create(const std::shared_ptr<const Grid::Properties>& properties,
                                             std::vector<uint8_t>& data)
    {
        return std::shared_ptr<Occupancy>(new Occupancy(properties, data));
    }


    /// @brief Returns the class type name for the Grid.
    const std::string& getTypeName() const
    {
        static const std::string name = "Occupancy";
        return name;
    }


protected:

    /// @brief Compares the ground truth to another vector
    /// @param experiment Vector of experimentally collected data to compare.
    /// @param [out] confusion Reference to a Confusion struct to store the results in.
    /// @return True of the vectors were the same size and the comparison was performed.
    bool compare(const std::vector<uint8_t>& experiment, Confusion& confusion) const
    {
        const size_t n = this->data.size();
        if (experiment.size() != n)
        {
            return false;
        }
        confusion.reset();

        for (size_t i = 0; i < n; ++i)
        {
            const uint8_t& truth       = this->data[i];
            const uint8_t& measurement = experiment[i];
            if (true_positive(truth, measurement))
            {
                ++confusion.tp;
            }
            else if (true_negative(truth, measurement))
            {
                ++confusion.tn;
            }
            else if (false_positive(truth, measurement))
            {
                ++confusion.fp;
            }
            else if (false_negative(truth, measurement))
            {
                ++confusion.fn;
            }
            else
            {
                ++confusion.uk;
            }
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


    uint8_t& at(Index idx)
    {
        return this->at(this->properties->at(idx));
    }


    uint8_t& at(size_t n)
    {
        return this->data.at(n);
    }


    uint8_t& operator[](Index idx)
    {
        return this->operator[](this->properties->operator[](idx));
    }


    uint8_t& operator[](size_t n)
    {
        return this->data[n];
    }


    /// @brief Ground truth data.
    std::vector<uint8_t> data;


private:
    /// @brief Private constructor to enforce use of shared pointers.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    explicit Occupancy(const std::shared_ptr<const Grid::Properties>& properties)
        : Grid(properties,
               DataType::UINT8_T)
    {
        this->data.resize(this->properties->getNumVoxels(), VoxelOccupancy::UNKNOWN);
    }


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param properties Shared, constant pointer to the Grid Properties to use.
    /// @throws GridPropertyError If the data vector length does not match the Grid Properties
    ///         number of voxels.
    /// @note   This uses `swap` to exchange the contents of the provided vector with the contents of
    ///         the internal data without copying data.
    explicit Occupancy(const std::shared_ptr<const Grid::Properties>& properties,
                           std::vector<uint8_t>& data)
        : Grid(properties,
               DataType::UINT8_T)
    {
        if (this->properties->getNumVoxels() != data.size())
        {
            throw GridPropertyError::DataVectorDoesNotMatch(this->properties->size, data.size());
        }
        this->data.swap(data);
    }


    /// @brief Tests the true-positive case where the voxel is occupied.
    /// @param truth Occupancy enumeration value for the voxel in the Ground Truth Occupancy Grid.
    /// @param measurement Experiments measurement for the same voxel in an Occupancy VoxelGrid.
    /// @return True if truth is OCCUPIED and the measurement is OCCUPIED.
    static bool true_positive(const uint8_t& truth, const uint8_t& measurement)
    {
        return measurement == VoxelOccupancy::OCCUPIED &&
               truth & VoxelOccupancy::TYPE_OCCUPIED;
    }


    /// @brief Tests the false-negative case where an occupied voxel is labeled as free or is unknown. 
    /// @param truth Occupancy enumeration value for the voxel in the Ground Truth Occupancy Grid.
    /// @param measurement Experiments measurement for the same voxel in an Occupancy VoxelGrid.
    /// @return True if truth is OCCUPIED and the measurement is FREE or UNKNOWN.
    ///         is OCCUPIED.
    static bool false_negative(const uint8_t& truth, const uint8_t& measurement)
    {
        return (measurement == VoxelOccupancy::FREE || measurement == VoxelOccupancy::UNKNOWN) &&
               truth & VoxelOccupancy::TYPE_OCCUPIED;
    }

    /// @brief Tests the false-positive case where a free voxel is labeled as occupied.
    /// @param truth Occupancy enumeration value for the voxel in the Ground Truth Occupancy Grid.
    /// @param measurement Experiments measurement for the same voxel in an Occupancy VoxelGrid.
    /// @return True if truth is FREE and the measurement is OCCUPIED.
    static bool false_positive(const uint8_t& truth, const uint8_t& measurement)
    {
        return measurement == VoxelOccupancy::OCCUPIED &&
               truth & VoxelOccupancy::TYPE_FREE;
    }


    /// @brief Tests the true-negative case where the voxel is free. 
    /// @param truth Occupancy enumeration value for the voxel in the Ground Truth Occupancy Grid.
    /// @param measurement Experiments measurement for the same voxel in an Occupancy VoxelGrid.
    /// @return True if truth is FREE and the measurement is FREE.
    static bool true_negative(const uint8_t& truth, const uint8_t& measurement)
    {
        return measurement == VoxelOccupancy::FREE &&
               truth & VoxelOccupancy::TYPE_FREE;
    }
};


} // namespace ground_truth
} // namespace metrics
} // namespace forge_scan


#endif // FORGE_SCAN_METRICS_GROUND_TRUTH_OCCUPANCY_HPP
