cmake_minimum_required(VERSION 3.9)
project(rangeOnly CXX C)
set(CXX_STANDARD 11)

SET(CMAKE_BUILD_TYPE "Debug")

find_package(GTSAM REQUIRED)

find_package(GTSAMCMakeTools)
find_package(nlohmann_json)

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

add_subdirectory(emulator)

add_library(libs STATIC "logging.h" "factors.h" "kernels.h")
set_target_properties(libs PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(libs PUBLIC gtsam gtsam_unstable emulator)

add_executable(main "main.cpp")
target_link_libraries(main PUBLIC gtsam gtsam_unstable nlohmann_json libs emulator)

set (testing 1)
if(testing)
    find_package(Catch2)
    enable_testing()
    add_subdirectory(tests)
endif()

