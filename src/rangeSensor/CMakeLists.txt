
add_library(
    sensor "dataEmulator.h" "realSource.h" "sensor.h" "base.h"
)

set_target_properties(sensor PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(sensor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sensor PUBLIC gtsam gtsam_unstable nlohmann_json)
