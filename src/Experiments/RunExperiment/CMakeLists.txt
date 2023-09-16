set(EXECUTABLE_NAME RunExperiment)
set(SOURCE_NAME     main.cpp)

add_executable(
    ${EXECUTABLE_NAME}
        ${SOURCE_NAME}
)
target_link_libraries(
    ${EXECUTABLE_NAME}
    PRIVATE
        ${INTERFACE_LIBRARY}
        ${DEFNITIONS_LIBRARY}
)
target_compile_options(
    ${EXECUTABLE_NAME}
    PRIVATE
        ${FORGE_SCAN_COMPILE_OPTIONS}
)