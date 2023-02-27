cmake_minimum_required(VERSION 3.9)
project(rangeOnly CXX C)
set(CXX_STANDARD 11)

SET(CMAKE_BUILD_TYPE "Debug")

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools)

find_package(nlohmann_json)

add_subdirectory(src)

add_executable(jsonDump "src/jsonDump.cpp")
target_link_libraries(jsonDump PUBLIC gtsam gtsam_unstable nlohmann_json utils libs sensor)

add_executable(calibration "src/calibration.cpp")
target_link_libraries(calibration PUBLIC gtsam gtsam_unstable nlohmann_json utils libs sensor)

add_executable(main "src/main.cpp")
target_link_libraries(main PUBLIC gtsam gtsam_unstable nlohmann_json utils libs sensor)

set (testing 1)
if(testing)
    find_package(Catch2)
    enable_testing()
    add_subdirectory(tests)
endif()

