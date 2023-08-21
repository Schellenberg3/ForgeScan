#ifndef FORGE_SCAN_METRICS_OCCUPANCY_CONFUSION_HPP
#define FORGE_SCAN_METRICS_OCCUPANCY_CONFUSION_HPP

#include <sstream>

#include "ForgeScan/Common/Definitions.hpp"

#include "ForgeScan/Metrics/Metric.hpp"
#include "ForgeScan/Metrics/GroundTruth/ExperimentVariants.hpp"
#include "ForgeScan/Metrics/GroundTruth/Occupancy.hpp"
#include "ForgeScan/Metrics/GroundTruth/TSDF.hpp"
#include "ForgeScan/Data/VoxelGrids/Occupancy.hpp"


namespace forge_scan {
namespace metrics {


class OccupancyConfusion : public Metric
{
public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Creates an OccupancyConfusion Metric.
    /// @param reconstruction Shared pointer to the Reconstruction that the Metric observes.
    /// @param ground_truth   The ground truth Occupancy Grid to compare Reconstruction data against.
    /// @param use_channel    The name of the Reconstruction channel to use. If this is an empty 
    ///                       string, the grid does not exist in the reconstruction, or the existing
    ///                       grid is the wrong type, then a default Occupancy Voxel Grid is created.
    /// @return Shared pointer to an an OccupancyConfusion Metric.
    /// @throws `std::invalid_argument` if the ground truth Occupancy Grid Properties are not equal
    ///         to those of the Reconstruction.
    static std::shared_ptr<OccupancyConfusion> create(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                                      const std::shared_ptr<const ground_truth::Occupancy>&   ground_truth,
                                                      const std::string& use_channel = "")
    {
        return std::shared_ptr<OccupancyConfusion>(new OccupancyConfusion(reconstruction, ground_truth, use_channel));
    }


    /// @brief Changes what ground truth the class uses.
    /// @param ground_truth The ground truth Occupancy Grid to compare Reconstruction data against.
    /// @returns True if the ground truth was changed. False if the Grid Properties were not equal
    ///          to those of the Reconstruction and the ground truth was not changed.
    bool setGroundTruth(const std::shared_ptr<const ground_truth::Occupancy>& ground_truth)
    {
        if (this->reconstruction->grid_properties->isEqual(ground_truth->properties))
        {
            this->ground_truth = ground_truth;
            return true;
        }
        return false;
    }


protected:
    // ***************************************************************************************** //
    // *                               PROTECTED CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param reconstruction Shared pointer to the Reconstruction that the Metric observes.
    /// @param ground_truth   The ground truth Occupancy Grid to compare Reconstruction data against.
    /// @param use_channel    The name of the Reconstruction channel to use. If this is an empty 
    ///                       string, the grid does not exist in the reconstruction, or the existing
    ///                       grid is the wrong type, then a default Occupancy Voxel Grid is created.
    /// @throws `std::invalid_argument` if the ground truth Occupancy Grid Properties are not equal
    ///          to those of the Reconstruction.
    explicit OccupancyConfusion(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                const std::shared_ptr<const ground_truth::Occupancy>&   ground_truth,
                                const std::string& use_channel = "")
        : Metric(reconstruction),
          channel_name(FS_METRIC_CHANNEL_PREFIX + this->getTypeName()),
          ground_truth(ground_truth)
    {
        this->throwIfGridPropertiesDoNotMatch();
        try
        {
            if (use_channel.empty()) throw std::invalid_argument("A Voxel Grid name must be provided.");
            auto voxel_grid = this->reconstruction->getChannel(use_channel);
            this->experiment = ground_truth::dynamic_cast_to_experimental_occupancy(voxel_grid);
        }
        catch(const std::exception&)
        {
            auto voxel_grid = data::Occupancy::create(this->reconstruction->grid_properties);
            this->addChannel(voxel_grid, this->getTypeName());
            this->experiment = voxel_grid;
        }
    }



    /// @brief Transforms the list of Confusion Matrix data into an Eigen matrix so it may be saved
    ///        in an HDF5 file.
    /// @return An Eigen matrix containing the data stored in the Confusion Matrix list.
    Eigen::Matrix<size_t, -1, -1> getConfusionAsMatrix() const
    {
        Eigen::Matrix<size_t, -1, -1> mat;
        mat.resize(this->confusion_list.size(), 6);
        size_t n = 0;
        for (const auto& item: this->confusion_list)
        {
            mat(n, 0) = item.second;
            mat(n, 1) = item.first.tp;
            mat(n, 2) = item.first.tn;
            mat(n, 3) = item.first.fp;
            mat(n, 4) = item.first.fn;
            mat(n, 5) = item.first.uk;
            ++n;
        }
        return mat;
    }


    /// @brief Verifies that the Grid Properties of the Reconstruction match those of the 
    ///        ground truth data.
    /// @throws `std::invalid_argument` if the ground truth Occupancy Grid Properties are not equal.
    void throwIfGridPropertiesDoNotMatch()
    {
        if ( !(this->reconstruction->grid_properties->isEqual(this->ground_truth->properties)) )
        {
            throw std::invalid_argument("TODO");
        }
    }



    // ***************************************************************************************** //
    // *                          PROTECTED VIRTUAL METHOD OVERRIDES                           * //
    // ***************************************************************************************** //


    void postUpdate(const size_t& reconstruction_update_count)
    {
        this->confusion_list.push_back({ground_truth::Confusion(), reconstruction_update_count});
        
        auto get_occupancy_data = [](auto&& experiment){
            return experiment->getOccupancyData();
        };

        this->ground_truth->compare(std::visit(get_occupancy_data, this->experiment),
                                    this->confusion_list.back().first);
    }


    void save(HighFive::File& file) const override final
    {
        static const std::vector<std::string> headers = {"update", "true positive", "true negative",
                                                         "false positive", "false negative", "unknown"};
        static const std::string hdf5_data_path = getDatasetPathHDF5(this->getTypeName());

        H5Easy::dump(file, hdf5_data_path, this->getConfusionAsMatrix());
        H5Easy::dumpAttribute(file, hdf5_data_path, "header", headers);
    }


    const std::string& getTypeName() const override final
    {
        static const std::string name = "OccupancyConfusion";
        return name;
    }



    // ***************************************************************************************** //
    // *                                PROTECTED CLASS MEMBERS                                * //
    // ***************************************************************************************** //


    /// @brief Records a pair of Confusion Matrix data and what Reconstruction update it came from.
    std::list<std::pair<ground_truth::Confusion, size_t>> confusion_list;

    /// @brief The ground truth Occupancy Grid to compare Reconstruction data against.
    std::shared_ptr<const ground_truth::Occupancy> ground_truth;

    /// @brief Reference to the Reconstruction Voxel Grid that this Metric uses.
    ground_truth::ExperimentOccupancy experiment;

    /// @brief Name for the channel the Metric makes.
    const std::string channel_name;
};


} // namespace metrics
} // namespace forge_scan



#endif // FORGE_SCAN_METRICS_OCCUPANCY_CONFUSION_HPP
