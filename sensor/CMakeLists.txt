
add_subdirectory(rangeSensor)

add_executable(calibration "calibration.cpp")
target_link_libraries(calibration PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)

add_executable(main "localising.cpp")
target_link_libraries(main PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)