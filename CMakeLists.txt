cmake_minimum_required(VERSION 3.9)
project(rangeOnly CXX C)
set(CXX_STANDARD 11)

STRING(REPLACE "-O2" "-Od" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
SET(CMAKE_BUILD_TYPE "Debug")

find_package(GTSAM REQUIRED)

find_package(GTSAMCMakeTools)
# find_package(nlohmann_json)

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

add_library(extralib STATIC emulator.h logging.h)
set_target_properties(extralib PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(extralib PUBLIC gtsam gtsam_unstable)


# add_library(factors STATIC factors.h)
# set_target_properties(factors PROPERTIES LINKER_LANGUAGE CXX)
# target_link_libraries(factors PUBLIC gtsam gtsam_unstable)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC gtsam gtsam_unstable extralib)
