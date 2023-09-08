## --------------------------------------------------------------------------------------------- ## 
## ------------------- DEFINE INTERFACE LIBRARY FOR COMPILIATION DEFINITIONS ------------------- ##
## --------------------------------------------------------------------------------------------- ## 

set(DEFNITIONS_LIBRARY ${PROJECT_NAME}Definitions)
add_library(
    ${DEFNITIONS_LIBRARY}
        INTERFACE
)
target_compile_definitions(
    ${DEFNITIONS_LIBRARY}
    INTERFACE
        FORGE_SCAN_DIR="${FORGE_SCAN_ROOT_DIR}"
        FORGE_SCAN_SHARE_DIR="${FORGE_SCAN_ROOT_DIR}/share"
)

add_subdirectory(Examples)

add_subdirectory(Experiments)
