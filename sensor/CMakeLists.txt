
add_subdirectory(rangeSensor)

add_executable(calibration "calibration.cpp")
target_link_libraries(calibration PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)

add_executable(jsonDump "jsonDump.cpp")
target_link_libraries(jsonDump PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)

add_executable(localise "localise.cpp")
target_link_libraries(localise PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)