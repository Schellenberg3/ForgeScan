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
        FORGE_SCAN_DIR=""
        FORGE_SCAN_SHARE_DIR="share"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
set(FORGE_SCAN_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Werror)
    message(STATUS "[ForgeScan::Info] Using GNU/Clang compiler options: ${FORGE_SCAN_COMPILE_OPTIONS}")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(STATUS "[ForgeScan::Info] Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
    message(STATUS "[ForgeScan::Info] target triplet: ${VCPKG_TARGET_TRIPLET}")
    # The data conversion or truncation warnings are suppressed. Most relate to initializations or intentional
    # conversions. Using `static_cast` would demonstrate intensionality to the compiler but cleaning this us
    # if a future to-do item.
    #     4244 - conversion from 'type1' to 'type2', possible loss of data. Implicit conversion issue.
    #     4267 - 'var' : conversion from 'size_t' to 'type', possible loss of data
    #     4305 - 'context' : truncation from 'type1' to 'type2'
    # Hiden class member warnings are suppressed. Intentional use of `this->` avoids the ambiguity in this project.
    # Perhaps this could be addressed in the future. But for now it is not important.
    #     4458 - declaration of 'identifier' hides class member
    # Constant contitional expressions are suppressed as these warnings come from the HighFive dependency.
    #     4127 - conditional expression is constant
    set(FORGE_SCAN_COMPILE_OPTIONS /W4 /wd4244 /wd4305 /wd4458 /wd4267 /wd4127)
    message(STATUS "[ForgeScan::Info] Using MSVC compiler options: ${FORGE_SCAN_COMPILE_OPTIONS}")
endif()


if(FORGE_SCAN_EXAMPLES)
  add_subdirectory(Examples)
endif()

if(FORGE_SCAN_EXPERIMENTS)
  add_subdirectory(Experiments)
endif()
