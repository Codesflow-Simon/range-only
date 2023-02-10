#pragma once

#include <list>
#include <nlohmann/json.hpp>

#include "base.h"
#include "dataEmulator.h"

using json = nlohmann::json;
using namespace std;

class JsonSensor {
  private:
    map<string, unsigned long int> keyTable;
    string tagID;
    unique_ptr<DataSource> data;

  public:

    JsonSensor(DataSource* data_) {
      data = unique_ptr<DataSource>(data_);
    }

    void updateTagKey(long unsigned int i) {
      keyTable[tagID] = i;
    }

    map<string,long unsigned int> getKeyTable() {
      return keyTable;
    }

    map<pair<string,string>, double> parseJson(json dataJson) {
      map<pair<string,string>, double> out;

      if (dataJson == json()) return map<pair<string,string>,double>();

      string firstID = dataJson["id"];
      json meas = dataJson["meas"];

      for (int i=0; i<meas["a"].size(); i++) {
        string secondID = meas["a"][i];
        double dist = meas["d"][i];

        pair<string,string> pair;
        pair.first = firstID;
        pair.second = secondID;

        out[pair] = dist; 
      }

      return out;
    }

    map<pair<string,string>, double> sample() {
      json dataJson = data->getJson();
      return parseJson(dataJson);
    }

    map<pair<string,string>, double> sampleA2a() {
      json dataJson = data->getJsonA2a();
      return parseJson(dataJson);
    }
};