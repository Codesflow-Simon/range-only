find_package(GTSAM REQUIRED)

find_package(GTSAMCMakeTools)

add_executable( data_emulatorTest " data_emulatorTest.cpp")
target_link_libraries( data_emulatorTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json libs sensor utils)
add_test(NAME  data_emulatorCTest COMMAND  data_emulatorTest)

add_executable(kernelTest "kernelTest.cpp")
target_link_libraries(kernelTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json libs sensor utils)
add_test(NAME kernelCTest COMMAND kernelTest)

add_executable(gaussianFactorTest "gaussianFactorTest.cpp")
target_link_libraries(gaussianFactorTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json libs sensor utils)
add_test(NAME gaussianFactorCTest COMMAND gaussianFactorTest)
