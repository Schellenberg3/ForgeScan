#include <ForgeScan/grid_processor.h>


namespace ForgeScan {


GridProcessor::GridProcessor(VoxelGrid& target)
{
    setTarget(target);
}

inline void GridProcessor::setTarget(VoxelGrid& new_target)
{
    target = &new_target;
    temp.resize(target->voxel_vector.size());
}

void inline GridProcessor::swap()
{
    target->voxel_vector.swap(temp);
}


void GridProcessor::operation(const std::function<void(const grid_idx&)>& operation)
{
    resetTempVector();
    grid_idx index(0, 0, 0);
    // Iterate fastest in X, then Y, and then Z and call the operation function on the voxel at that index.
    for (size_t z = 0, z_max = target->properties.grid_size[2]; z < z_max; ++z) {
        index[2] = z;
        for (size_t y = 0, y_max = target->properties.grid_size[1]; y < y_max; ++y) {
            index[1] = y;
            for (size_t x = 0, x_max = target->properties.grid_size[0]; x < x_max; ++x) {
                index[0] = x;
                operation(index);
            }
        }
    }
    swap();
}

/*
void GridProcessor::dilate(const grid_idx& element, const int& n)
{
    std::vector<grid_idx> neighbors(6, grid_idx(0, 0, 0));
    vector_idx element_vidx = 0;

    this->voxel_grid->toVector(element, element_vidx);
    int known_neighbor_count = this->voxel_grid->at(element_vidx).updates != 0 ? 1 : 0;

    this->voxel_grid->get_6(element, neighbors);

    for (const auto& neighbor : neighbors)
        if (this->voxel_grid->valid(neighbor) && this->voxel_grid->at(neighbor).updates != 0)
            ++known_neighbor_count;

    if (known_neighbor_count >= n)
        this->temp->at(element_vidx).updates = 1;
}


void GridProcessor::erode(const grid_idx& element, const int& n)
{
    std::vector<grid_idx> neighbors(6, grid_idx(0, 0, 0));
    vector_idx element_vidx = 0;

    this->voxel_grid->toVector(element, element_vidx);
    // int known_neighbor_count = this->voxel_grid->at(element_vidx) == 0 ? 1 : 0;
    int known_neighbor_count = 1;
    if (this->voxel_grid->at(element_vidx).updates != 0)
        known_neighbor_count = 0;

    this->voxel_grid->get_6(element, neighbors);

    for (const auto& neighbor : neighbors)
        if (this->voxel_grid->valid(neighbor) && this->voxel_grid->at(neighbor).updates == 0)
            ++known_neighbor_count;

    if (known_neighbor_count >= n)
        this->temp->at(element_vidx).updates = 0;
    else  // Otherwise, stay the same
        this->temp->at(element_vidx).updates = this->voxel_grid->at(element_vidx).updates;
}
*/

} // ForgeScan
