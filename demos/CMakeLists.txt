set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard to use")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# TEST_GRID
add_executable(test_grid test_grid.cpp)
target_link_libraries(
    test_grid
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)


# SIM_SCAN
add_executable(sim_scan sim_scan.cpp)
target_link_libraries(
    sim_scan
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)


# TEST_VOXEL
add_executable(test_voxel test_voxel.cpp)
target_link_libraries(
    test_voxel
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)


# TEST_TRAVERSAL
add_executable(test_traversal test_traversal.cpp)
target_link_libraries(
    test_traversal
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
)


# DEMO_POLICY
add_executable(demo_policy demo_policy.cpp)
target_link_libraries(
    demo_policy
    PUBLIC
        ${FORGESCAN_LIBRARY_NAME}
        Eigen3::Eigen
        HighFive
)
