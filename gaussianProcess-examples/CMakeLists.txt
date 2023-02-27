
add_executable(jsonDump "./jsonDump.cpp")
target_link_libraries(jsonDump PUBLIC gtsam gtsam_unstable nlohmann_json utils gaussian range)