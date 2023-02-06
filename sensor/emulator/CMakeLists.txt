
add_library(
    emulator
    "${CMAKE_CURRENT_SOURCE_DIR}/sensorEmulator.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/cameraEmulator.h"
)

set_target_properties(emulator PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(emulator PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(emulator PUBLIC gtsam gtsam_unstable nlohmann_json base)
