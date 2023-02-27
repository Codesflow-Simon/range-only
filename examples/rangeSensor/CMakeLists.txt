
add_library(
    range "dataEmulator.h" "realSource.h" "base.h"
)

set_target_properties(range PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(range PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(range PUBLIC gtsam gtsam_unstable nlohmann_json utils)
