cmake_minimum_required(VERSION 3.9)
project(rangeOnly CXX C)
set(CXX_STANDARD 11)

SET(CMAKE_BUILD_TYPE "Debug")

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools)

find_package(nlohmann_json)

add_subdirectory(src)
add_subdirectory(examples)

set (testing 1)
if(testing)
    find_package(Catch2)
    enable_testing()
    add_subdirectory(tests)
endif()

