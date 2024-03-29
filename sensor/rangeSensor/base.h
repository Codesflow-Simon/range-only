#pragma once

#include <iostream>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <list>
#include <map>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>

#include <nlohmann/json.hpp>

using namespace std; 
using namespace Eigen;
using namespace gtsam;

typedef nlohmann::json json;
typedef Eigen::Matrix<double,3,1> Vector3d;
typedef PinholeCamera<Cal3_S2> Camera;


/**
 * @brief Model of anchor or tag object
 */
class Anchor {
  public:
    string ID;
    Vector3d location;
    unsigned long int key;

    /**
     * @brief Default construction, set ID to empty string and location to zero
     * 
     */
    Anchor() {
      ID = "";
      location = Vector3d();
    }

    /**
     * @brief Construct a new Anchor object
     * 
     * @param location_
     * @param ID_ 
     */
    Anchor(Vector3d location_, string ID_) {
      location = location_;
      ID = ID_;
    }

    /**
     * @brief tests anchor equality
     * 
     * @param other 
     * @return true 
     * @return false 
     */
    bool operator ==(const Anchor& other) const {
      const string otherID = other.ID;
      return otherID == ID;
    }

    /**
     * @brief tests anchor equality
     * 
     * @param other 
     * @return true 
     * @return false 
     */
    bool equals(const Anchor& other) const {
    return ID == other.ID;
    }

    /**
     * @brief Implements order for std::maps
    */
    bool operator <(const Anchor& other) const {
      return stoi(ID) < stoi(other.ID);
    }

    /**
     * @brief Converts a anchor to human-readable string format
     * 
     * @return string
     */
    string to_string_() {
      return "Anchor(\"" + ID + "\", x=" + to_string(location.x()) + ", y=" + to_string(location.y()) + ", z=" + to_string(location.z()) + ")\n";
    }
};

/**
 * @brief a generic template for a sensor that makes measurements using JSON objects
*/
class DataSource {    
  private:
    map<string, Key> keyTable; // Could be replaced with a good hash function -- needs to manage X(i)'s
    string tagID = "0b05";

  public:
    virtual json getJson() = 0;
    virtual void sendA2a() = 0;
    virtual void updateTimeIndex(int time) {};

    /**
     * @brief Constructor with data source
    */
    void initKeyTable(string prefix = "../") {

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
      json dataJson = getJson();
      return parseJson(dataJson);
    }
};

/**
 * @brief a generic template for both the camera emulator and physical camera
*/
class CameraWrapper {
  protected:
    Camera* camera;
    Cal3_S2::shared_ptr params;

  public:

    virtual Camera* getCamera() = 0;
    virtual Cal3_S2::shared_ptr  getParams() = 0;

    virtual Point2 sample(Point3 tag) = 0;
};