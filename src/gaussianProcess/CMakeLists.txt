add_library(libs STATIC "graph_adders.h" "kernels.h")
set_target_properties(libs PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(libs PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(libs PUBLIC gtsam gtsam_unstable utils)