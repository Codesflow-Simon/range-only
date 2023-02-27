
add_subdirectory(rangeSensor)

add_executable(jsonDump "src/jsonDump.cpp")
target_link_libraries(jsonDump PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)

add_executable(calibration "src/rangeSensor-calibration.cpp.cpp")
target_link_libraries(calibration PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)

add_executable(main "src/rangeSensor-localising.cpp.cpp")
target_link_libraries(main PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)