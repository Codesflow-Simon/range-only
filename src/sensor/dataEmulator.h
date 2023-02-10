#pragma once

#include <iostream>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <vector>
#include <random>
#include <string>
#include <fstream>

#include <nlohmann/json.hpp>

#include "util.h"
#include "base.h"

using namespace std;
using namespace Eigen;

using json = nlohmann::json;

class Emulator : public DataSource {
  private:
    json path;
    json anchorsJson;

    vector<Anchor> anchors;
    vector<Anchor>::iterator it = anchors.begin();

    // State variables
    double error;
    int numData = 200;
    int a2aSent = -1;
    int timeIndex = 0;
    double time = 0;
    double dt = 0.05;
    string tagID = "0b05";

    // Random generators
    default_random_engine generator;
    normal_distribution<double> dist;

    // Sampling methods
    vector<double> sampleAsVector(Vector3 tag) {
      int idx = 0;
      vector<double> output = vector<double>(anchors.size());
      for (auto const& anchor : anchors) {
        double noise = dist(generator);
        double distance = distanceBetween(anchor.location, tag);

        output.at(idx) = distance + noise;
          
        idx++;
      }
      return output;
    }

    // Json construction methods
    void constructConstants(json* base) {
      (*base)["id"] = tagID;
      (*base)["acc"] = {0,0,-9.81};
      (*base)["gyro"] = {0.0, 0.0, 0.0};
      (*base)["mag"] = {0, 0, 21};
      (*base)["ts"] = (double)time;
    }

    void getTagData(json* base, Vector3 tag) {

      constructConstants(base);

      json meas = json();
      vector<double> measurement = sampleAsVector(tag);

      auto ids = vector<string>();
      vector<string>::iterator id_it = ids.begin();
      for (auto const& anchor: anchors) {
        ids.insert(id_it, anchor.ID);
        id_it = ids.end();
      }

      meas["a"] = ids;
      meas["d"] = measurement;      
      (*base)["meas"] = meas;
    }
    
    void getA2aData(json* base, Vector3 tag) {
      Anchor subject = anchors.at(a2aSent);

      constructConstants(base);           

      json meas = json();
      auto measurement = sampleAsVector(subject.location);

      auto ids = vector<string>();
      vector<string>::iterator it = ids.begin();
      for (auto const& anchor: anchors) {
        if (anchor == subject) continue;
          ids.insert(it, anchor.ID);
          it = ids.end();
        }

      meas["a"] = ids;
      meas["d"] = removeZeros(measurement);  
      (*base)["meas"] = meas;
      a2aSent--;
    }

  public:
    Emulator() {
      std::ifstream f("../path.json");
      path = json::parse(f);
      std::ifstream g("../anchors.json");
      anchorsJson = json::parse(g);

      for (int i=0; i<8; i++) {
        string ID = to_string(i);
        Vector3d loc = Vector3d(anchorsJson[i]["x"], anchorsJson[i]["y"], anchorsJson[i]["z"]);
        anchors.push_back(Anchor(loc, ID));
      }
    }

    json getJson() {
      if (timeIndex >= numData) return json();
      Vector3 tag = Vector3( stod((string)path[timeIndex]["x"]), stod((string)path[timeIndex]["y"]), 
                             stod((string)path[timeIndex]["z"]) );

      json output = json();
      if (a2aSent==-1) {
        getTagData(&output, tag);
      } else {
        getA2aData(&output, tag);
      }
      timeIndex++;
      return output;
    }

    json getJsonA2a() {
      sendA2a();
      return getJson();
    }

    void sendA2a() {
      a2aSent = anchors.size()-1;
    }

    void clearA2a() {
      a2aSent = -1;
    }
};