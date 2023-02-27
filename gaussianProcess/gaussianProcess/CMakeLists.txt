add_library(gaussian STATIC "gaussian.h" "kernels.h")
set_target_properties(gaussian PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(gaussian PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(gaussian PUBLIC gtsam gtsam_unstable utils)