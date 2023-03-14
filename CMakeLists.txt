CMAKE_MINIMUM_REQUIRED (VERSION 3.7.2)
# The minimum version is set by the VSCode CMake Tools extrension requirement
# of CMake Server mode for Debugging.For more, see:
#   https://vector-of-bool.github.io/docs/vscode-cmake-tools/debugging.html#target-debugging-and-launching

PROJECT ( ForgeScan )

SET ( CMAKE_BUILD_TYPE Debug)
SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET ( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET ( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET ( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET ( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )
SET ( FORGESCAN_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)


FIND_PACKAGE (HDF5 REQUIRED COMPONENTS C CXX)
FIND_PACKAGE (HighFive REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED NO_MODULE)
### Why `NO_MODULE` ono Eigen3? I don't know
### Ask their CMake guide:
###     https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html


### Note that finding and linking to PCL has SUCKED. I'm not sure
### What's wrong with it... 
###     https://stackoverflow.com/questions/66669735/
# SET(PCL_DIR "/path/to/PCLConfig.cmake")
# SET(PCL_INCLUDE_DIRS /usr/include/pcl-1.10)
# SET(PCL_DIR /usr/include/pcl-1.10)
# FIND_PACKAGE (PCL REQUIRED COMPONENTS common io)
# MESSAGE ( "PCL DIR:       ${PCL_DIR}" )
# MESSAGE ( "PCL VERSION:   ${PCL_VERSION}" )
# MESSAGE ( "PLC INCLUDE DIR:   ${PCL_INCLUDE_DIRS}" )
# MESSAGE ( "PCL LIBRARY DIR:   ${PCL_LIBRARY_DIRS}" )
# MESSAGE ( "PCL LIBRARIES:     ${PCL_LIBRARIES}" )


INCLUDE_DIRECTORIES (
    ${HDF5_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
    ${HDF5_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${FORGESCAN_INCLUDE_DIRS}
)

LINK_DIRECTORIES ( 
    ${OCTOMAP_LIBRARY_DIRS}
    ${EIGEN3_LIBRARY_DIRS}
#   ${PCL_LIBRARY_DIRS}
)

LINK_LIBRARIES ( 
    ${EIGEN3_LIBRARIES}
)

# ADD_DEFINITIONS( ${PCL_DEFINITIONS} )


ADD_EXECUTABLE (
    test_grid 

    src/test_voxel_grid.cpp
    src/voxel_grid.cpp
    src/grid_processor.cpp
    src/voxel_element.cpp
    src/grid_traversal.cpp
)
TARGET_LINK_LIBRARIES(test_grid HighFive Eigen3::Eigen)
### Why the `Library::Component` structure? I don't know...
### See the following on CMake namespaces:
###     https://stackoverflow.com/questions/60605281/

ADD_EXECUTABLE (
    sim_sphere_scan 

    src/sim_sphere_scan.cpp
    src/sim_sensor_reading.cpp
    src/voxel_grid.cpp
    src/voxel_element.cpp
    src/grid_traversal.cpp
)
TARGET_LINK_LIBRARIES(sim_sphere_scan HighFive Eigen3::Eigen)


ADD_EXECUTABLE (
    test_voxel_element

    src/test_voxel_element.cpp
    src/voxel_element.cpp
    src/voxel_grid.cpp
    src/grid_traversal.cpp
)
TARGET_LINK_LIBRARIES(test_voxel_element Eigen3::Eigen)

ADD_EXECUTABLE (
    test_traversal

    src/test_traversal.cpp
    src/voxel_element.cpp
    src/voxel_grid.cpp
    src/grid_traversal.cpp
)
TARGET_LINK_LIBRARIES(test_voxel_element Eigen3::Eigen)
