#ifndef FORGESCAN_GRID_PROCESSOR_H
#define FORGESCAN_GRID_PROCESSOR_H

#include <ForgeScan/voxel_grid.h>

#include <vector>
#include <memory>
#include <functional>
#include <Eigen/Geometry>


typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;

class VoxelGrid;  // Promise to compiler that the VoxelGrid class will be defined later


class GridProcessor {
    private:
        /// Temporary vector that is the destination for changes when running a filter process 
        std::shared_ptr<std::vector<uint8_t>> temp;

        /// Pointer to the VoxelGrid to act upon.
        /// Using pointer to allow nullptr as a valid input.
        VoxelGrid *voxel_grid;

        /// @brief Informs the class what VoxelGrid it is acting on.
        /// @param target The new target to add
        bool inline set_new_target(VoxelGrid* target) {
            if (target != nullptr && target != this->voxel_grid) {
                this->voxel_grid = target;
                return true;
            } else if (this->voxel_grid == nullptr) // Both given target and stored are nullptr
                throw std::invalid_argument("Cannot process without providing a valid VoxelGrid");
            return false;
        }

        /// @brief Ensures that proper memory is allocated to the temporary vector
        /// @note This does not modify the vector contents or size, only how much space it has.
        void inline setup_temp_vector();


        /// @brief Resets the temporary vector to the provided value
        /// @param val Value to reset each temp vector element to. Default 0.
        void inline reset_temp(const size_t& val = 0) {
            // TODO: While resetting to 0 after each operation an valid option, there are be cases where
            //       it is more optimal to have the current value of the voxel_grid. Essentially, for erosion
            //       and dilation it would mean we only need to flip when the threshold it met. 
            //       However, future changes - especially moving away from categorization - may mean the effort
            //       to make this change would be wasted. For now, this note is just a reminder for future me.
            std::fill(this->temp->begin(), this->temp->end(), val);
        }


        /// @brief Performs an elementwise operation, like erosion or dilation.
        /// @param operation Function to perform. Function must take a constant reference to an element.
        /// @param target Pointer to a VoxelGrid. If this is something other than null and not whatever 
        ///               the class is currently pointint to, then it will resize the vector for this new target.
        void elementwise_operation(const std::function<void(const Vector3ui&)>& operation, VoxelGrid* target);


        /// @brief Erodes the element in the temporary vector at the provided location. 
        /// @param element Location to erode.
        /// @param n Number of un-sensed (zero) neighbors. If n or greater are un-sensed, then the element is set to 0.
        void erode_element(const Vector3ui& element, const int& n);


        /// @brief Dilates the element in the temporary vector at the provided location. 
        /// @param element Location to dilate.
        /// @param n Number of sensed (non-zero) neighbors. If n or greater are sense, then the element is set to 1.
        void dilate_element(const Vector3ui& element, const int& n);


    public:
        GridProcessor();
        
        GridProcessor(VoxelGrid& target);


        /// @brief Performs erosion of the grid. 
        /// @param n Number of un-sensed (zero) neighbors. If n or greater are un-sensed, then the element is set to 0.
        /// @param target Optional VoxelGrid target to set or switch the GridProcessor to.
        void inline erode(const int& n, VoxelGrid* target = nullptr) {
            elementwise_operation([this, n](const Vector3ui& element) {this->erode_element(element, n);}, target);
        }


        /// @brief Performs dilation on the grid.
        /// @param n Number of sensed (non-zero) neighbors. If n or greater are sense, then the element is set to 1.
        /// @param target Optional VoxelGrid target to set or switch the GridProcessor to.
        void inline dilate(const int& n, VoxelGrid* target = nullptr) {
            elementwise_operation([this, n](const Vector3ui& element) {this->dilate_element(element, n);}, target);
        }
};

#endif // FORGESCAN_GRID_PROCESSOR_H
