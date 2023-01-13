find_package(GTSAM REQUIRED)

find_package(GTSAMCMakeTools)

add_executable(cameraTests "distanceFactorTest.cpp")
target_link_libraries(cameraTests PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable emulator libs)
add_test(NAME cameraCTest COMMAND cameraTests)

