find_package(GTSAM REQUIRED)

find_package(GTSAMCMakeTools)

add_executable(dataEmulatorTest "dataEmulatorTest.cpp")
target_link_libraries(dataEmulatorTest PRIVATE Catch2::Catch2WithMain gtsam gtsam_unstable nlohmann_json libs sensor)
add_test(NAME dataEmulatorCTest COMMAND dataEmulatorTest)
