
add_library(
    sensor
    "${CMAKE_CURRENT_SOURCE_DIR}/random_tools.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/dataEmulator.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/realSource.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/sensor.h"
)

set_target_properties(sensor PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(sensor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sensor PUBLIC gtsam gtsam_unstable nlohmann_json)
