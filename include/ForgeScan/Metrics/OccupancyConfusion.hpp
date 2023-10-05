#ifndef FORGE_SCAN_METRICS_OCCUPANCY_CONFUSION_HPP
#define FORGE_SCAN_METRICS_OCCUPANCY_CONFUSION_HPP

#include <sstream>

#include "ForgeScan/Common/Definitions.hpp"

#include "ForgeScan/Metrics/Metric.hpp"
#include "ForgeScan/Metrics/GroundTruth/ExperimentVariants.hpp"
#include "ForgeScan/Metrics/GroundTruth/Occupancy.hpp"


namespace forge_scan {
namespace metrics {


class OccupancyConfusion : public Metric
{
public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Creates an OccupancyConfusion Metric.
    /// @param reconstruction Shared pointer to the `data::Reconstruction` that the Metric observes.
    /// @param ground_truth   The `ground_truth::Occupancy` Grid to compare `data::Reconstruction` data against.
    /// @param use_channel    The name of the `data::Reconstruction` channel to use. If this is an empty
    ///                       string, the grid does not exist in the `data::Reconstruction`, or the existing
    ///                       grid is the wrong type, then a default `data::Binary` `data::VoxelGrid` is created.
    /// @return Shared pointer to an an OccupancyConfusion Metric.
    /// @throws GridPropertyError if the `ground_truth::Occupancy` `Grid::Properties` are not equal
    ///                           to those of the `data::Reconstruction`.
    /// @throws BadVoxelGridDownCast if the channel from `use_channel` may not be cast to one of
    ///                              the supported `ground_truth::ExperimentOccupancy` types.
    static std::shared_ptr<OccupancyConfusion> create(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                                      const std::shared_ptr<const ground_truth::Occupancy>& ground_truth,
                                                      const std::string& use_channel = "")
    {
        return std::shared_ptr<OccupancyConfusion>(new OccupancyConfusion(reconstruction, ground_truth, use_channel));
    }


    /// @brief Changes what ground truth the class uses.
    /// @param ground_truth The `ground_truth::Occupancy` Grid to compare Reconstruction data against.
    /// @returns True if the ground truth was changed. False if the `Grid::Properties` were not equal
    ///          to those of the `data::Reconstruction` and the ground truth was not changed.
    bool setGroundTruth(const std::shared_ptr<const ground_truth::Occupancy>& ground_truth)
    {
        if (this->reconstruction->grid_properties->isEqual(ground_truth->properties))
        {
            this->ground_truth = ground_truth;
            return true;
        }
        return false;
    }


    /// @return Help message for constructing a OccupancyConfusion with ArgParser.
    static std::string helpMessage()
    {
        /// TODO: Return an fill this in.
        return "TODO: OccupancyConfusion help message";
    }


    static const std::string type_name;


protected:
    // ***************************************************************************************** //
    // *                               PROTECTED CLASS METHODS                                 * //
    // ***************************************************************************************** //


    /// @brief Private constructor to enforce use of shared pointers.
    /// @param reconstruction Shared pointer to the `data::Reconstruction` that the Metric observes.
    /// @param ground_truth   The `ground_truth::Occupancy` Grid to compare `data::Reconstruction` data against.
    /// @param use_channel    The name of the `data::Reconstruction` channel to use. If this is an empty
    ///                       string, the grid does not exist in the `data::Reconstruction`, or the existing
    ///                       grid is the wrong type, then a default `data::Binary` `data::VoxelGrid` is created.
    /// @throws GridPropertyError if the ground truth Occupancy Grid Properties are not equal
    ///                           to those of the Reconstruction.
    /// @throws BadVoxelGridDownCast if the channel from `use_channel` may not be cast to one of
    ///                              the supported `ground_truth::ExperimentOccupancy` types.
    explicit OccupancyConfusion(const std::shared_ptr<data::Reconstruction>& reconstruction,
                                const std::shared_ptr<const ground_truth::Occupancy>& ground_truth,
                                const std::string& use_channel = "")
        : Metric(reconstruction,
                 OccupancyConfusion::getMapName(use_channel)),
          channel_name(FS_METRIC_CHANNEL_PREFIX + OccupancyConfusion::type_name),
          ground_truth(ground_truth)
    {
        this->throwIfGridPropertiesDoNotMatch();
        if (use_channel.empty())
        {
            auto voxel_grid = data::Binary::create(this->reconstruction->grid_properties);
            this->addChannel(voxel_grid, channel_name);
            this->experiment = voxel_grid;
        }
        else
        {
            auto voxel_grid = this->reconstruction->getChannelView(use_channel);
            this->experiment = ground_truth::dynamic_cast_to_experimental_occupancy(voxel_grid);
        }
    }


    static std::string getMapName(const std::string& use_channel)
    {
        if (use_channel.empty())
        {
            return OccupancyConfusion::type_name;
        }
        return OccupancyConfusion::type_name + "_" + use_channel;
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
    /// @throws GridPropertyError if the ground truth Occupancy Grid Properties are not equal.
    void throwIfGridPropertiesDoNotMatch()
    {
        if (this->reconstruction->grid_properties->isEqual(this->ground_truth->properties))
        {
            return;
        }
        throw GridPropertyError::PropertiesDoNotMatch("Reconstruction", "Ground Truth Occupancy");
    }



    // ***************************************************************************************** //
    // *                          PROTECTED VIRTUAL METHOD OVERRIDES                           * //
    // ***************************************************************************************** //


    void postUpdate(const size_t& update_count) override final
    {
        this->confusion_list.push_back({ground_truth::Confusion(), update_count});

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
        
        const std::string hdf5_data_path = getDatasetPathHDF5(this->map_name);
        H5Easy::dump(file, hdf5_data_path, this->getConfusionAsMatrix());
        H5Easy::dumpAttribute(file, hdf5_data_path, "header", headers);
    }


    const std::string& getTypeName() const override final
    {
        return OccupancyConfusion::type_name;
    }



    // ***************************************************************************************** //
    // *                                PROTECTED CLASS MEMBERS                                * //
    // ***************************************************************************************** //


    /// @brief Records a pair of Confusion Matrix data and what Reconstruction update it came from.
    std::list<std::pair<ground_truth::Confusion, size_t>> confusion_list;

    /// @brief Name for the channel the Metric makes.
    const std::string channel_name;

    /// @brief The ground truth Occupancy Grid to compare Reconstruction data against.
    std::shared_ptr<const ground_truth::Occupancy> ground_truth;

    /// @brief Reference to the Reconstruction VoxelGrid that this Metric uses.
    ground_truth::ExperimentOccupancy experiment;
};


/// @brief String for the class name.
const std::string OccupancyConfusion::type_name = "OccupancyConfusion";


} // namespace metrics
} // namespace forge_scan



#endif // FORGE_SCAN_METRICS_OCCUPANCY_CONFUSION_HPP
