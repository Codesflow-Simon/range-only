#pragma once

#include <iostream>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <vector>
#include <random>
#include <string>
#include <fstream>
#include <map>

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
    map<string, double> sampleAsMap(Vector3 tag, string exclude="") {

      map<string, double> out;

      for (auto const& anchor : anchors) {
        if (anchor.ID == exclude) continue;

        double noise = dist(generator);
        double distance = distanceBetween(anchor.location, tag);

        out[anchor.ID] = distance + noise;
      }
      return out;
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
      map<string,double> measurement = sampleAsMap(tag);
      
      auto ids = vector<string>();
      auto measVec = vector<double>();
      for (auto pair : measurement) {
        ids.push_back(pair.first);
        measVec.push_back(pair.second);
      }

      meas["a"] = ids;
      meas["d"] = measVec;      
      (*base)["meas"] = meas;
    }
    
    void getA2aData(json* base, Vector3 tag) {
      Anchor subject = anchors.at(a2aSent);

      constructConstants(base);           

      json meas = json();
      auto measurement = sampleAsMap(subject.location, subject.ID);

      auto ids = vector<string>();
      auto measVec = vector<double>();
      for (auto pair : measurement) {
        ids.push_back(pair.first);
        measVec.push_back(pair.second);
      }

      meas["a"] = ids;
      meas["d"] = measVec;  
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