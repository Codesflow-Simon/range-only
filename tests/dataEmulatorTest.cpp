#include <catch2/catch_test_macros.hpp>
#include <math.h>
#include <iostream>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

#include "dataEmulator.h"
#include "random_tools.h"

TEST_CASE("Data Emulator 1") {
    Emulator source;

    source.set_error(0);
    string tagID = "0b05";

    std::ifstream f("../path.json");
    json path = json::parse(f);

    std::ifstream g("../anchors.json");
    json anchors = json::parse(g);

    for (int i=0; i<200; i++) {

        json data = source.getJson();
        REQUIRE(data["id"] == tagID);

        Vector3 tag = Vector3( stod((string)path[i]["x"]), stod((string)path[i]["y"]), 
                               stod((string)path[i]["z"]) );
        
        int j=0;
        while (j < data["meas"]["a"].size()) {
            Vector3 anchor = Vector3( stod((string)anchors[j]["x"]), stod((string)anchors[j]["y"]), 
                                      stod((string)anchors[j]["z"]) );
            REQUIRE((string)anchors[j]["ID"] == data["meas"]["a"][j]);
            REQUIRE(abs(distanceBetween(tag, anchor) - (double)data["meas"]["d"][j]) < 1e-8);
        }
    }



}