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


if(FORGE_SCAN_BUILD_EXAMPLES)
  add_subdirectory(Examples)
endif()

if(FORGE_SCAN_BUILD_EXPERIMENTS)
  add_subdirectory(Experiments)
endif()
