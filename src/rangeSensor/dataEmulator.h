#pragma once

#include <iostream>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <vector>
#include <random>
#include <string>
#include <fstream>
#include <map>

#include <nlohmann/json.hpp>

#include "random_tools.h"
#include "base.h"

using namespace std;
using namespace Eigen;

using json = nlohmann::json;

/**
 * @brief Derives from DataSource, uses a simulated environment to create JSON objects
*/
class Emulator : public DataSource {
  private:
    json path;
    json anchorsJson;

    vector<Anchor> anchors;
    vector<Anchor>::iterator it = anchors.begin();

    // State variables
    double error = 0.01;
    int numData = 1000;
    int a2aSent = -1;
    int timeIndex = 0;
    double time = 0;
    double dt = 0.05;
    string tagID = "0b05";

    // Random generators
    default_random_engine generator;
    normal_distribution<double> dist;

    // Sampling methods
    map<string, double> sampleAsMap(Vector3d tag, string exclude="") {

      map<string, double> out;

      for (auto const& anchor : anchors) {
        if (anchor.ID == exclude) continue;

        double noise = dist(generator) * error;
        double distance = distanceBetween(anchor.location, tag);

        out[anchor.ID] = distance + noise;
      }
      return out;
    }

    
    /**
     * @brief adds some constants to a json
     * @param json* json to add to
    */
    void constructConstants(json* base) {
      (*base)["id"] = tagID;
      (*base)["acc"] = {0,0,-9.81};
      (*base)["gyro"] = {0.0, 0.0, 0.0};
      (*base)["mag"] = {0, 0, 21};
      (*base)["ts"] = (double)time;
    }

    /**
     * @brief fetches the json including distance measurements to tag
     * @param json* json to add to
     * @param tag location of tag to base measurements
    */
    void getTagData(json* base, Vector3d tag) {

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
    
    /**
     * @brief fetches the json including distance measurements pairwise between all anchors
     * @param json* json to add to
    */
    void getA2aData(json* base) {
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
    /**
     * @brief constructor with path
     * @param string path of physical sensors
    */
    Emulator(string prefix = "../") {
      std::ifstream f(prefix+"./data/path.json");
      path = json::parse(f);
      std::ifstream g(prefix+"anchors.json");
      anchorsJson = json::parse(g);
    

      for (json anchor : anchorsJson) {
        string ID = anchor["ID"];
        Vector3d loc = Vector3d(anchor["x"], anchor["y"], anchor["z"]);
        Anchor anc(loc, ID);
        anc.key = anchor["key"];
        anchors.push_back(anc);
      }
    }

    /**
     * @brief sets measurement error
     * @param string path of physical sensors
    */
    void set_error(double error_) {
      error = error_;
    }

    /**
     * @brief required method, get the measurement json from the emulator
     * @return json
    */
    json getJson() {
      if (timeIndex >= numData) return json();
      Vector3d tag = Vector3d( stod((string)path[timeIndex]["x"]), stod((string)path[timeIndex]["y"]), 
                             stod((string)path[timeIndex]["z"]) );

      json output = json();
      if (a2aSent==-1) {
        getTagData(&output, tag);
      } else {
        getA2aData(&output);
      }
      return output;
    }

    /**
     * @brief use to keep time in sync with the main program
     * @param int time
    */
    void updateTimeIndex(int time) {
      timeIndex = time;
    }

    /**
     * @brief required method, get the measurement json from the emulator and sendsA2a command
     * @return json
    */
    json getJsonA2a() {
      sendA2a();
      return getJson();
    }
  
    /**
      * @brief sends a2a command
     */
    void sendA2a() {
      a2aSent = anchors.size()-1;
    }
  
    /**
      * @brief undoes any sent a2a
     */
    void clearA2a() {
      a2aSent = -1;
    }
};