# Archive

This directory contains code that was developed during the project to test new functionality or demonstrate concepts. The contents are outdated and are only saved as quick-reference material. Use or view at your own risk.

To use any of this code, move the file to the `/src` directory and add the relevant section of the following to the package CMake file:

```CMake
ADD_EXECUTABLE (
    sample_sphere 
    
    src/sphere_octomap_and_pc.cpp
)
# TARGET_LINK_LIBRARIES(sample_sphere ${PCL_LIBRARIES})


ADD_EXECUTABLE ( 
    csv_from_octree_pc 

    src/csv_from_octree_pc.cpp
)


ADD_EXECUTABLE (
    csv_from_voxel_pc

    src/csv_from_voxel_pc.cpp
    src/voxel_grid.cpp
)
TARGET_LINK_LIBRARIES(csv_from_voxel_pc HighFive Eigen3::Eigen)


ADD_EXECUTABLE (
    depth_camera_sim 

    src/depth_camera_sim.cpp
    src/voxel_grid.cpp
)
TARGET_LINK_LIBRARIES(depth_camera_sim HighFive Eigen3::Eigen)
```
