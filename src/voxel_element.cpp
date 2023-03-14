#include <ForgeScan/voxel_element.h>


void inline init_voxel_element(VoxelElement& element, const VoxelElementUpdate& update)
{
    if (update.new_dist)
    {
        element.min_dist = *update.new_dist;
        element.avg_dist = *update.new_dist;
    }
    if (update.new_cent)    element.cent = *update.new_cent;
    if (update.new_norm)    element.norm = *update.new_norm;
    if (update.new_density) element.density = *update.new_density;
}


void update_voxel_element(VoxelElement& element, const VoxelElementUpdate& update)
{
    if (element.get_view_count() == 0)
    {
        init_voxel_element(element, update);
    } else {
        if (update.new_dist)
        {
            element.update_min_dist(*update.new_dist);
            element.update_avg_std_dist(*update.new_dist);
        }

        if (update.new_cent)
            element.update_cent(*update.new_cent);

        if (update.new_norm)
            element.update_norm(*update.new_norm);

        if (update.new_density)
            element.update_density(*update.new_density);
    }
    element.inc_views();
}
