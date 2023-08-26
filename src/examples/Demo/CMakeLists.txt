set(EXECUTABLE_NAME Demo)
set(SOURCE_NAME ${EXECUTABLE_NAME}.cpp)
add_executable(
    ${EXECUTABLE_NAME}
        ${SOURCE_NAME}
)
target_include_directories(
    ${EXECUTABLE_NAME}
    PUBLIC
        ${FORGESCAN_INCLUDE_DIRS}
)
target_link_libraries(
    ${EXECUTABLE_NAME}
    PRIVATE
        Eigen3::Eigen
        HighFive
        ${OpenCV_LIBS}
)
set_target_properties(
    ${EXECUTABLE_NAME}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)
target_compile_options(
    ${EXECUTABLE_NAME}
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)