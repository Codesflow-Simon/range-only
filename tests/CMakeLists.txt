
add_executable( dataEmulatorTest "dataEmulatorTest.cpp")
target_link_libraries( dataEmulatorTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json utils gaussian range)
add_test(NAME  dataEmulatorTest COMMAND  dataEmulatorTest)

add_executable(kernelTest "kernelTest.cpp")
target_link_libraries(kernelTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json utils gaussian range)
add_test(NAME kernelCTest COMMAND kernelTest)

add_executable(gaussianFactorTest "gaussianFactorTest.cpp")
target_link_libraries(gaussianFactorTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json utils gaussian range)
add_test(NAME gaussianFactorCTest COMMAND gaussianFactorTest)
