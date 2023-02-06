cmake_minimum_required(VERSION 3.9)
project(rangeOnly CXX C)
set(CXX_STANDARD 11)

SET(CMAKE_BUILD_TYPE "Debug")

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools)

find_package(nlohmann_json)

find_package(gpmp2)
include_directories(${gpmp2_INCLUDE_DIR})

add_subdirectory(sensor)

add_executable(main "src/main.cpp")
target_link_libraries(main PUBLIC gtsam gtsam_unstable nlohmann_json libs physical base)

set (testing 1)
if(testing)
    find_package(Catch2)
    enable_testing()
    add_subdirectory(tests)
endif()

