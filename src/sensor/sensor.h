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

    map<pair<string,string>, double> sample() {
      map<pair<string,string>, double> out;

      json dataJson = data->getJson();
      string firstID = dataJson["id"];

      json meas = dataJson["meas"];

      for (json measurement : meas.items()) {
        cout << measurement << endl;
        string secondID = measurement["a"];
        double dist = measurement["d"];

        pair<string,string> pair;
        pair.first = firstID;
        pair.second = secondID;

        out[pair] = dist;
      }

      return out;
    }

    map<pair<string,string>, double> sampleA2a() {
      return data->getJsonA2a();
    }
};