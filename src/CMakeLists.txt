# FORGESCAN_LIB

add_library(
    ${FORGESCAN_LIBRARY_NAME}
    STATIC
        TSDF/grid_io.cpp
        TSDF/traversal.cpp
        TSDF/processor.cpp
        Metrics/sensor_record_io.cpp
)
target_link_libraries(
    ${FORGESCAN_LIBRARY_NAME}
    PRIVATE
        Eigen3::Eigen
        HighFive
)
target_include_directories(
    ${FORGESCAN_LIBRARY_NAME}
    PUBLIC
        ${FORGESCAN_INCLUDE_DIRS})
target_compile_definitions(
    ${FORGESCAN_LIBRARY_NAME}
    PUBLIC
        FORGESCAN_SHARE_DIR="${FORGESCAN_SHARE_DIR}"
        FORGESCAN_SHARE_PARAVIEW_DIR="${FORGESCAN_SHARE_DIR}/ParaView"
)
set_target_properties(
    ${FORGESCAN_LIBRARY_NAME}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)
