find_package(GTSAM REQUIRED)

find_package(GTSAMCMakeTools)

add_executable(gaussianFactorTest "gaussianFactorTest.cpp")
target_link_libraries(gaussianFactorTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json libs sensor)
add_test(NAME gaussianFactorCTest COMMAND gaussianFactorTest)

add_executable(functionTests "kernelTest.cpp")
target_link_libraries(functionTests PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json libs sensor)
add_test(NAME functionTests COMMAND functionTests)

