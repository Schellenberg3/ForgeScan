#include <ForgeScan/grid_processor.h>


GridProcessor::GridProcessor()
{
    this->voxel_grid = nullptr;
    this->temp = std::make_shared<std::vector<VoxelElement>>();
}


GridProcessor::GridProcessor(VoxelGrid& target)
{
    this->voxel_grid = &target;
    this->temp = std::make_shared<std::vector<VoxelElement>>();
    this->setup_temp_vector();
}


void inline GridProcessor::setup_temp_vector()
{
    // Despite being inline we define this here to avoid invalid usage 
    // of an incomplete type of VoxelGrid
    size_t grid_size = this->voxel_grid->grid->size();
    if ( this->temp->size() != grid_size)
        this->temp = std::make_shared<std::vector<VoxelElement>>(grid_size, VoxelElement());
}


void GridProcessor::elementwise_operation(const std::function<void(const grid_idx&)>& operation, VoxelGrid* target)
{
    if (this->set_new_target(target)) this->setup_temp_vector();
    this->reset_temp();

    grid_idx current_gidx(0, 0, 0);
    size_t current_vidx = 0;

    for (size_t z = 0, z_max = this->voxel_grid->size[2]; z < z_max; ++z)
    {
        current_gidx[2] = z;
        for (size_t y = 0, y_max = this->voxel_grid->size[1]; y < y_max; ++y)
        {
            current_gidx[1] = y;
            for (size_t x = 0, x_max = this->voxel_grid->size[0]; x < x_max; ++x)
            {
                current_gidx[0] = x;
                operation(current_gidx);
            }
        }
    }
    this->voxel_grid->grid.swap(this->temp);
}


void GridProcessor::dilate_element(const grid_idx& element, const int& n)
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


void GridProcessor::erode_element(const grid_idx& element, const int& n)
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
