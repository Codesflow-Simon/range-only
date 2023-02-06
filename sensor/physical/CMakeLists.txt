
add_library(
    physical
    "${CMAKE_CURRENT_SOURCE_DIR}/emulator.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/sensor.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/util.h"
)

set_target_properties(physical PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(physical PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(physical PUBLIC gtsam gtsam_unstable nlohmann_json base)
