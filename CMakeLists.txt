
cmake_minimum_required(VERSION 1.0)
project(rangeOnly CXX C)
set(CXX_STANDARD 11)

find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

add_library(extralib SHARED emulator.hpp logging.hpp)
set_target_properties(extralib PROPERTIES LINKER_LANGUAGE CXX)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC gtsam nlohmann_json extralib)

