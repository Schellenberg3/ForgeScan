cmake_minimum_required(VERSION 3.12)

project(
    ForgeScan
    VERSION 0.3
    DESCRIPTION
    "Policies for autonomous, iterative, and fast voxelized geometric reconstruction with simulated depth cameras."
    LANGUAGES C CXX
)
include(GNUInstallDirs)
set(FORGE_SCAN_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})


## --------------------------------------------------------------------------------------------- ##
## ------------------------------------ COMPILATION OPTIONS ------------------------------------ ##
## --------------------------------------------------------------------------------------------- ##

option(FORGE_SCAN_EXAMPLES         "Enable compilation of example executables"     ON)
option(FORGE_SCAN_EXPERIMENTS      "Enable compilation of experiment executables"  ON)
option(FORGE_SCAN_BUILD_DOCS       "Enable building project documentation"         ON)
option(FORGE_SCAN_ONLY_BUILD_DOCS  "Builds only the project documentation"         OFF)


if(FORGE_SCAN_BUILD_DOCS)
    add_subdirectory(doc)
    if (FORGE_SCAN_ONLY_BUILD_DOCS)
        # If only the Docs are built then return early instead
        # of searching for dependencies that won't be needed.
        return()
    endif()
endif()

## --------------------------------------------------------------------------------------------- ##
## ---------------------------------- FIND EXTERNAL PACKAGES ----------------------------------- ##
## --------------------------------------------------------------------------------------------- ##

find_package(Open3D REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Prefere using the submodule of HighFive, if it exists.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive/CMakeLists.txt")
    # Set a number of HighFive options to OFF. Prevents examples and unit tests from compiling when
    # building this project.
    set(HIGHFIVE_UNIT_TESTS OFF CACHE BOOL "Enable unit tests(requires Catch2 to be present)")
    set(HIGHFIVE_BUILD_DOCS OFF CACHE BOOL "Enable documentation building")
    set(HIGHFIVE_EXAMPLES   OFF CACHE BOOL "Compile examples")
    set(USE_BOOST           OFF CACHE BOOL "Enable Boost Support")

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/deps/HighFive)
    message(STATUS "[ForgeScan::Info] Using local sub-module for HighFive.")
else()
    find_package(HighFive REQUIRED)
    message(STATUS "[ForgeScan::Info] Using external libaray for HighFive.")
endif()


## --------------------------------------------------------------------------------------------- ##
## ----------------------------- DEFINE PROJECT INTERFACE LIBRARY ------------------------------ ##
## --------------------------------------------------------------------------------------------- ##

# Ensure C++ 17 or greater is used within this project.
if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
elseif(CMAKE_CXX_STANDARD < 17)
    message(FATAL "[ForgeScan] ForgeScan requires C++ 17 or greater.")
endif()

# Set output directories for build.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)


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
        Open3D::Open3D
)
set_target_properties(
    ${INTERFACE_LIBRARY}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

# Add project executable targets.
add_subdirectory(src)



## --------------------------------------------------------------------------------------------- ##
## ----------------------------- DEFINE PROJECT INSTALLATION STEPS ----------------------------- ##
## --------------------------------------------------------------------------------------------- ##

set(FORGE_SCAN_INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(
    TARGETS
        ${INTERFACE_LIBRARY}
    EXPORT
        ${PROJECT_NAME}Targets
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/CMake/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION
        ${FORGE_SCAN_INSTALL_DESTINATION}
)
write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    EXPORT
        ${PROJECT_NAME}Targets
    FILE
        ${PROJECT_NAME}Targets.cmake
    NAMESPACE
        ${PROJECT_NAME}::
    DESTINATION
        ${FORGE_SCAN_INSTALL_DESTINATION}
)
install(
    FILES
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION
       ${FORGE_SCAN_INSTALL_DESTINATION}
)
install(
    DIRECTORY
        ${PROJECT_SOURCE_DIR}/include/
    DESTINATION
        include
)

message(STATUS ${FORGE_SCAN_INSTALL_DESTINATION})

