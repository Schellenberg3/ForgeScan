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

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
set(FORGE_SCAN_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Werror)
    message(STATUS "[ForgeScan::Info] Using GNU/Clang compiler options: ${FORGE_SCAN_COMPILE_OPTIONS}")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(STATUS "[ForgeScan::Info] Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
    message(STATUS "[ForgeScan::Info] target triplet: ${VCPKG_TARGET_TRIPLET}")
    set(FORGE_SCAN_COMPILE_OPTIONS /W4)
    message(STATUS "[ForgeScan::Info] Using MSVC compiler options: ${FORGE_SCAN_COMPILE_OPTIONS}")
    message(WARNING "[ForgeScan::Info] MSVC support is experimental.")
endif()


if(FORGE_SCAN_EXAMPLES)
  add_subdirectory(Examples)
endif()

if(FORGE_SCAN_EXPERIMENTS)
  add_subdirectory(Experiments)
endif()
