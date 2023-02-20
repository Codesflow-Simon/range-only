#pragma once

#include <list>
#include <nlohmann/json.hpp>
#include <gtsam/inference/Symbol.h>

#include "base.h"
#include "dataEmulator.h"

using json = nlohmann::json;
using namespace std;

class JsonSensor {
  private:
    map<string, unsigned long int> keyTable;
    string tagID = "0b05";
    unique_ptr<DataSource> data;
    int uniqueInt = 0;

  public:

    JsonSensor(DataSource* data_) {
      data = unique_ptr<DataSource>(data_);

      std::ifstream g("../data/anchors.json");
      json anchorsJson = json::parse(g);

      for (json anchor : anchorsJson) {
        keyTable[anchor["ID"]] = anchor["key"];
      }
    }

    void updateTagKey(long unsigned int i) {
      keyTable[tagID] = i;
    }

    map<string,long unsigned int> getKeyTable() {
      return keyTable;
    }

    map<pair<Key,Key>, double> parseJson(json dataJson) {
      map<pair<Key,Key>, double> out;

      if (dataJson == json()) return map<pair<Key,Key>,double>();

      string firstID = dataJson["id"];
      json meas = dataJson["meas"];

      for (int i=0; i<meas["a"].size(); i++) {
        string secondID = meas["a"][i];

        double dist = meas["d"][i];

        pair<Key,Key> pair;
        pair.first = keyTable[firstID];
        pair.second = keyTable[secondID];

        out[pair] = dist; 
      }

      return out;
    }

    map<pair<Key,Key>, double> sample() {
      json dataJson = data->getJson();
      return parseJson(dataJson);
    }

    map<pair<Key,Key>, double> sampleA2a() {
      json dataJson = data->getJsonA2a();
      return parseJson(dataJson);
    }
};