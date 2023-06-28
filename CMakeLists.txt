cmake_minimum_required(VERSION 3.7.2)
# The minimum version is set by the VSCode CMake Tools extrension requirement. See:
#   https://vector-of-bool.github.io/docs/vscode-cmake-tools/debugging.html#target-debugging-and-launching

project(
    ForgeScan
    VERSION 0.1
    DESCRIPTION
    "Policies for autonomous, iterative, and fast voxelized geometric reconstruction with simulated depth cameras."
    LANGUAGES CXX
)

# Turn off meaningless boost versioning warning.
set(Boost_NO_WARN_NEW_VERSIONS 1)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message ("[ForgeScan::Info] Using GNU/Clang compiler options")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message ("[ForgeScan::Info] Using MSVC compiler options")
    message( "[ForgeScan::Info] Toolchain file: ${CMAKE_TOOLCHAIN_FILE}" )
    message( "[ForgeScan::Info] target triplet: ${VCPKG_TARGET_TRIPLET}" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif()

# LOCAL DIRECTORY VARIABLES
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(FORGESCAN_INCLUDE_DIRS         ${CMAKE_SOURCE_DIR}/include)
set(FORGESCAN_SHARE_DIR            ${CMAKE_SOURCE_DIR}/share)
set(FORGESCAN_SHARE_PARAVIEW_DIR   ${FORGESCAN_SHARE_DIR}/paraview)


###########################################
## LOCATE DEPENDENCIES AND INFORM LINKER ##
###########################################

# LOCATE HIGHFIVE AND SET ITS COMPILE OPTIONS
# Set a number of HighFive options to off by default. Essentially prevents their examples and
# unit tests from being compiled automatically when building.
if(EXISTS "${CMAKE_SOURCE_DIR}/deps/HighFive/CMakeLists.txt")
    set(HIGHFIVE_UNIT_TESTS OFF CACHE BOOL "Enable unit tests(requires Catch2 to be present)")
    set(HIGHFIVE_BUILD_DOCS OFF CACHE BOOL "Enable documentation building")
    set(HIGHFIVE_EXAMPLES   OFF CACHE BOOL "Compile examples")
    set(USE_BOOST           OFF CACHE BOOL "Enable Boost Support")

    add_subdirectory(${CMAKE_SOURCE_DIR}/deps/HighFive)
    set(HIGHFIVE_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/deps/HighFive/include)
    message( "[ForgeScan::Info] Using local sub-module for HighFive." )
else()
    find_package(HighFive REQUIRED)
    message( "[ForgeScan::Info] Searching externally for HighFive." )
endif()

# LOCATE DEPENDENCIES INSTALLED WITH APT OR VCPKG
# find_package(PCL    REQUIRED COMPONENTS common io)
find_package(HDF5   REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)


##########################################
## DEFINE PROJECT'S COMPILATION TARGETS ##
##########################################

# FORGESCAN_LIB
set(FORGESCAN_LIBRARY_NAME ${PROJECT_NAME})
add_library(
    ${FORGESCAN_LIBRARY_NAME}
    STATIC
        src/TSDF/grid_io.cpp
        src/TSDF/traversal.cpp
        src/TSDF/processor.cpp
        src/Metrics/sensor_record_io.cpp
)
target_link_libraries(
    ${FORGESCAN_LIBRARY_NAME}
    PRIVATE
        Eigen3::Eigen
        HighFive
)
target_include_directories(${FORGESCAN_LIBRARY_NAME} PRIVATE ${FORGESCAN_INCLUDE_DIRS})


# TEST_GRID
add_executable(
    test_grid
    demos/test_grid.cpp
)
target_link_libraries(
    test_grid
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)
target_compile_definitions(
    test_grid
    PUBLIC
        FORGESCAN_SHARE_DIR="${FORGESCAN_SHARE_DIR}"
        FORGESCAN_SHARE_PARAVIEW_DIR="${FORGESCAN_SHARE_DIR}/ParaView"
)
target_include_directories(test_grid PUBLIC ${FORGESCAN_INCLUDE_DIRS})


# SIM_SCAN
add_executable(
    sim_scan
    demos/sim_scan.cpp
)
target_link_libraries(
    sim_scan
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)
target_compile_definitions(
    sim_scan
    PUBLIC
        FORGESCAN_SHARE_DIR="${FORGESCAN_SHARE_DIR}"
        FORGESCAN_SHARE_PARAVIEW_DIR="${FORGESCAN_SHARE_DIR}/ParaView"
)
target_include_directories(sim_scan PUBLIC ${FORGESCAN_INCLUDE_DIRS})


# TEST_VOXEL
add_executable(
    test_voxel
    demos/test_voxel.cpp
)
target_link_libraries(
    test_voxel
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)
target_include_directories(test_voxel PUBLIC ${FORGESCAN_INCLUDE_DIRS})


# TEST_TRAVERSAL
add_executable(
    test_traversal
    demos/test_traversal.cpp
)
target_link_libraries(
    test_traversal
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)
target_compile_definitions(
    test_traversal
    PUBLIC
        FORGESCAN_SHARE_DIR="${FORGESCAN_SHARE_DIR}"
        FORGESCAN_SHARE_PARAVIEW_DIR="${FORGESCAN_SHARE_DIR}/ParaView"
)
target_include_directories(test_traversal PUBLIC ${FORGESCAN_INCLUDE_DIRS})


# DEMO_POLICY
add_executable(
    demo_policy
    demos/demo_policy.cpp
)
target_link_libraries(
    demo_policy
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
        HighFive
)
target_compile_definitions(
    demo_policy
    PUBLIC
        FORGESCAN_SHARE_DIR="${FORGESCAN_SHARE_DIR}"
        FORGESCAN_SHARE_PARAVIEW_DIR="${FORGESCAN_SHARE_DIR}/ParaView"
)
target_include_directories(demo_policy PUBLIC ${FORGESCAN_INCLUDE_DIRS})
