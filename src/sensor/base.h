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
 * 
 */
class Anchor {
  public:
    string ID;
    Vector3d location;

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
     * @brief TEST_CASEs anchor equality
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
     * @brief TEST_CASEs anchor equality
     * 
     * @param other 
     * @return true 
     * @return false 
     */
    bool equals(const Anchor& other) const {
    return ID == other.ID;
    }

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
 * @brief a generic template for both the sensor emulator and physical sensors
*/
class DataSource {    
  public:
    virtual json getJson();
    virtual json sampleA2a();
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