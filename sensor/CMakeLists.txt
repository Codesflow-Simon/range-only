
add_library(
    base
    "${CMAKE_CURRENT_SOURCE_DIR}/base.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/random_tools.h"
)

set_target_properties(base PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(base PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(base PUBLIC gtsam gtsam_unstable nlohmann_json)

add_subdirectory(emulator)
add_subdirectory(physical)