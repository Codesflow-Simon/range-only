#include <catch2/catch_test_macros.hpp>
#include <math.h>
#include <iostream>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

#include "dataEmulator.h"
#include "sensor.h"
#include "random_tools.h"

TEST_CASE("Data Emulator 1") {
    Emulator source("../../data/");

    source.set_error(0);
    string tagID = "0b05";

    std::ifstream f("../../data/path.json");
    json path = json::parse(f);

    std::ifstream g("../../data/anchors.json");
    json anchors = json::parse(g);

    for (int i=0; i<200; i++) {
        source.updateTimeIndex(i);
        json data = source.getJson();

        REQUIRE(data["id"] == tagID);

        Vector3 tag = Vector3( stod((string)path[i]["x"]), stod((string)path[i]["y"]), 
                               stod((string)path[i]["z"]) );
        
        int j=0;
        while (j < data["meas"]["a"].size()) {
            Vector3 anchor = Vector3( anchors[j]["x"], anchors[j]["y"], anchors[j]["z"] );
            REQUIRE((string)anchors[j]["ID"] == data["meas"]["a"][j]);
            REQUIRE(abs(distanceBetween(tag, anchor) - (double)data["meas"]["d"][j]) < 1e-8);
            j++;
        }
    }
}

TEST_CASE("Sensor Emulator comparison test 1") {
    Emulator* source = new Emulator("../../data/");
    JsonSensor* sensor = new JsonSensor(source, "../../data/");

    source->set_error(0);
    string tagID = "0b05";

    std::ifstream f("../../data/path.json");
    json path = json::parse(f);

    std::ifstream g("../../data/anchors.json");
    json anchors = json::parse(g);

    for (int i=0; i<200; i++) {
        source->updateTimeIndex(i);
        json data1 = source->getJson();
        auto data2 = sensor->sample();

        REQUIRE(data1["id"] == tagID);

        Vector3 tag = Vector3( stod((string)path[i]["x"]), stod((string)path[i]["y"]), 
                               stod((string)path[i]["z"]) );
        
        int j=0;
        for(auto pair : data2) {
            Vector3 anchor = Vector3( anchors[j]["x"], anchors[j]["y"], anchors[j]["z"] );

            REQUIRE((string)anchors[j]["ID"] == data1["meas"]["a"][j]);
            REQUIRE(abs(distanceBetween(tag, anchor) - (double)data1["meas"]["d"][j]) < 1e-8);

            REQUIRE(abs(distanceBetween(tag, anchor) - pair.second) < 1e-8);
            j++;
        }
    }
}