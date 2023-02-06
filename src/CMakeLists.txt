
add_library(libs STATIC "logging.h" "factors.h" "kernels.h")
set_target_properties(libs PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(libs PUBLIC gtsam gtsam_unstable physical)