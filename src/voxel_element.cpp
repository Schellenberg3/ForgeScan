#include <ForgeScan/voxel_element.h>

void update_voxel_element(VoxelElement& element, const VoxelElementUpdate& update)
{
    if (update.new_dist)
        element.update_dist_average(*update.new_dist);

    if (update.new_cent)
        element.update_cent(*update.new_cent);

    if (update.new_norm)
        element.update_norm(*update.new_norm);

    if (update.new_density)
        element.update_density(*update.new_density);

    element.inc_views();
}
