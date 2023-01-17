find_package(GTSAM REQUIRED)

find_package(GTSAMCMakeTools)

add_executable(factorTests "distanceFactorTest.cpp")
target_link_libraries(factorTests PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable emulator libs)
add_test(NAME factorCTest COMMAND factorTests)

add_executable(functionTests "kernelTest.cpp")
target_link_libraries(functionTests PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable emulator libs)
add_test(NAME functionTests COMMAND functionTests)

