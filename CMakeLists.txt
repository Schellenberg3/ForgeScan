cmake_minimum_required(VERSION 3.7.2)

project(
    ForgeScan
    VERSION 0.2
    DESCRIPTION
    "Policies for autonomous, iterative, and fast voxelized geometric reconstruction with simulated depth cameras."
    LANGUAGES C CXX
)
include(GNUInstallDirs)
set(FORGE_SCAN_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})


# ----- Print compiler and toolchain information -----

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "Using GNU/Clang compiler options")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(STATUS "Using MSVC compiler options")
    message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
    message(STATUS "target triplet: ${VCPKG_TARGET_TRIPLET}")
    message(FATAL_ERROR "MSVC is not supported. Project will not be built.")
    # The project can likely operate on windows. But more testing is needed and
    # some CMake code will be needed to set compiler warnings and settings
    # for each target as they are tested and based on the build variant.
endif()


# ----- Find external packages -----

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)


# ----- Find HighFive dependency, prefering submodule, if available  -----

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive/CMakeLists.txt")
    # Set a number of HighFive options to off by default. Essentially prevents their examples and
    # unit tests from being compiled automatically when building.
    set(HIGHFIVE_UNIT_TESTS OFF CACHE BOOL "Enable unit tests(requires Catch2 to be present)")
    set(HIGHFIVE_BUILD_DOCS OFF CACHE BOOL "Enable documentation building")
    set(HIGHFIVE_EXAMPLES   OFF CACHE BOOL "Compile examples")
    set(USE_BOOST           OFF CACHE BOOL "Enable Boost Support")

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive)
    set(HIGHFIVE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive/include)
    message(STATUS "[ForgeScan::Info] Using local sub-module for HighFive.")
else()
    find_package(HighFive REQUIRED)
    message(STATUS "[ForgeScan::Info] Using external libaray for HighFive.")
endif()


# ----- Setup project compilation -----

set(INTERFACE_LIBRARY ${PROJECT_NAME})
add_library(
    ${INTERFACE_LIBRARY}
        INTERFACE
)
add_library(
    ${PROJECT_NAME}::${PROJECT_NAME}
    ALIAS
        ${INTERFACE_LIBRARY}
)

target_include_directories(
    ${INTERFACE_LIBRARY}
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    ${INTERFACE_LIBRARY}
    INTERFACE
        ${OpenCV_LIBS}
        Eigen3::Eigen
        HighFive
)
set_target_properties(
    ${INTERFACE_LIBRARY}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

add_subdirectory(src)


# ----- Install this projects -----
# -----         TODO          -----
