add_library(utils STATIC "logging.h" "random_tools.h")
set_target_properties(utils PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(utils PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(utils PUBLIC gtsam gtsam_unstable)