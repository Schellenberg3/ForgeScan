cmake_minimum_required(VERSION 3.7.2)

project(
    ForgeScan
    VERSION 0.1
    DESCRIPTION
    "Policies for autonomous, iterative, and fast voxelized geometric reconstruction with simulated depth cameras."
    LANGUAGES C CXX
)


# ----- Print compiler and toolchain information -----
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "[ForgeScan::Info] Using GNU/Clang compiler options")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(STATUS "[ForgeScan::Info] Using MSVC compiler options")
    message(STATUS "[ForgeScan::Info] Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
    message(STATUS "[ForgeScan::Info] target triplet: ${VCPKG_TARGET_TRIPLET}")
endif()


# ----- Set local varialbes -----
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(FORGESCAN_INCLUDE_DIRS         ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(FORGESCAN_SHARE_DIR            ${CMAKE_CURRENT_SOURCE_DIR}/share)
set(FORGESCAN_SHARE_PARAVIEW_DIR   ${FORGESCAN_SHARE_DIR}/paraview)

set(FORGESCAN_LIBRARY_NAME ${PROJECT_NAME})


# ----- Find required dependencies -----
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive/CMakeLists.txt")
    # Set a number of HighFive options to off by default. Essentially prevents their examples and
    # unit tests from being compiled automatically when building.
    set(HIGHFIVE_UNIT_TESTS OFF CACHE BOOL "Enable unit tests(requires Catch2 to be present)")
    set(HIGHFIVE_BUILD_DOCS OFF CACHE BOOL "Enable documentation building")
    set(HIGHFIVE_EXAMPLES   OFF CACHE BOOL "Compile examples")
    set(USE_BOOST           OFF CACHE BOOL "Enable Boost Support")

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive)
    set(HIGHFIVE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive/include)
    message("[ForgeScan::Info] Using local sub-module for HighFive.")
else()
    find_package(HighFive REQUIRED)
    message("[ForgeScan::Info] Searching externally for HighFive.")
endif()

find_package(Eigen3 REQUIRED NO_MODULE)


# ----- Add project subdirectories -----
add_subdirectory(src)
add_subdirectory(demos)
