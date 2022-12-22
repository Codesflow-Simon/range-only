cmake_minimum_required(VERSION 1.0)
project(rangeOnly CXX C)
set(CXX_STANDARD 11)

cmake_policy(SET CMP0003 NEW)

find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
find_package(nlohmann_json REQUIRED)

message(CMAKE_CXX_COMPILER)

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

add_library(extralib SHARED emulator.h logging.h)
set_target_properties(extralib PROPERTIES LINKER_LANGUAGE CXX)

add_library(factors SHARED factors.h)
set_target_properties(factors PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(factors PRIVATE gtsam)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC gtsam nlohmann_json extralib factors)
