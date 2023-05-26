cmake_minimum_required(VERSION 3.7.2)
# The minimum version is set by the VSCode CMake Tools extrension requirement. See:
#   https://vector-of-bool.github.io/docs/vscode-cmake-tools/debugging.html#target-debugging-and-launching

project(ForgeScan)

# Turn off meaningless boost versioning warning.
set(Boost_NO_WARN_NEW_VERSIONS 1)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message ("[ForgeScan::Info] Using GNU/Clang compiler options")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message ("[ForgeScan::Info] Using GNU/Clang compiler options")
    message( "[ForgeScan::Info] Toolchain file: ${CMAKE_TOOLCHAIN_FILE}" )
    message( "[ForgeScan::Info] target triplet: ${VCPKG_TARGET_TRIPLET}" )
endif()

# LOCAL DIRECTORY VARIABLES
set(BASE_DIR ${CMAKE_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin)
set(FORGESCAN_INCLUDE_DIRS         ${BASE_DIR}/include)

###########################################
## LOCATE DEPENDENCIES AND INFORM LINKER ##
###########################################

# LOCATE HIGHFIVE AND SET ITS COMPILE OPTIONS
# Set a number of HighFive options to off by default. Essentially prevents their examples and
# unit tests from being compiled automatically when building all available targets.
set(HIGHFIVE_UNIT_TESTS OFF CACHE BOOL "Enable unit tests(requires Catch2 to be present)")
set(HIGHFIVE_BUILD_DOCS OFF CACHE BOOL "Enable documentation building")
set(HIGHFIVE_EXAMPLES   OFF CACHE BOOL "Compile examples")

add_subdirectory(${BASE_DIR}/deps/HighFive)
set(HIGHFIVE_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/deps/HighFive/include)

# LOCATE DEPENDENCIES INSTALLED WITH APT
find_package(PCL    REQUIRED COMPONENTS common io)
find_package(HDF5   REQUIRED COMPONENTS C CXX)
find_package(Eigen3 REQUIRED NO_MODULE)
### See Eigne CMake guide:
###     https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html

include_directories(
    ${FORGESCAN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${HDF5_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${HIGHFIVE_INCLUDE_DIRS}
)

link_directories(
    ${EIGEN3_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${HDF5_LIBRARY_DIRS}
)

link_libraries(
    ${EIGEN3_LIBRARIES}
    ${HDF5_LIBRARIES}
)

add_definitions(
    ${PCL_DEFINITIONS}
    ${HDF5_DEFINITIONS}
)

##########################################
## DEFINE PROJECT'S COMPILATION TARGETS ##
##########################################

# TEST_GRID
add_executable(
    test_grid 

    demos/test_voxel_grid.cpp
    src/voxel_grid.cpp
    src/voxel_grid_traversal.cpp
    src/grid_processor.cpp
)
target_link_libraries(
    test_grid
    
    HighFive
    Eigen3::Eigen
)

# SIM_SPHERE_SCAN
add_executable(
    sim_scan

    demos/sim_scan.cpp
    src/voxel_grid.cpp
    src/voxel_grid_traversal.cpp
)
target_link_libraries(
    sim_scan
    
    HighFive
    Eigen3::Eigen
)

# TEST_VOXEL_ELEMENT
add_executable(
    test_voxel_element

    demos/test_voxel_element.cpp
    src/voxel_grid.cpp
    src/voxel_grid_traversal.cpp
)
target_link_libraries(
    test_voxel_element
    
    Eigen3::Eigen
)

# TEST_TRAVERSAL
add_executable(
    test_traversal

    demos/test_traversal.cpp
    src/voxel_grid.cpp
    src/voxel_grid_traversal.cpp
)
target_link_libraries(
    test_traversal

    Eigen3::Eigen
)
