#pragma once

#include <list>
#include <nlohmann/json.hpp>

#include "base.h"
#include "dataEmulator.h"

using json = nlohmann::json;
using namespace std;
using Key = unsigned long int;

/**
 * @brief JsonSensor takes input from a DataSource and parses it into c++ structs. JsonSensor also manages the assignment of unique keys to sensors.
*/
class JsonSensor {
  private:
    map<string, Key> keyTable; // Could be replaced with a good hash function -- needs to manage X(i)'s
    string tagID = "0b05";
    unique_ptr<DataSource> data;
    int uniqueInt = 0;

  public:
    /**
     * @brief Constructor with data source
    */
    JsonSensor(DataSource* data_, string prefix = "../") {
      data = unique_ptr<DataSource>(data_);

      std::ifstream g(prefix+"anchors.json");
      json anchorsJson = json::parse(g);

      for (json anchor : anchorsJson) {
        keyTable[anchor["ID"]] = anchor["key"];
      }
    }

    /**
     * @brief Keep tag key up to date
    */
    void updateTagKey(Key i) {
      keyTable[tagID] = i;
    }

    /**
     * @brief Parses a json from the data source
    */
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


    /**
     * @brief required method, gets data
    */
    map<pair<Key,Key>, double> sample() {
      json dataJson = data->getJson();
      return parseJson(dataJson);
    }

    /**
     * @brief sends a2a
    */
    void sendA2a() {
      data->sendA2a();
    }
};