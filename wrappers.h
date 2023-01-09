#include <gtsam/geometry/PinholeCamera.h>
#include <Eigen/Dense>
#include <map>
#include <gtsam/geometry/Cal3_S2.h>

#pragma once

using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3_S2> Camera;

typedef Eigen::Matrix<double,3,1> Vector3;

/**
 * @brief Model of anchor or tag object
 * 
 */
class Anchor {
  public:
    string ID;
    Vector3 location;

    /**
     * @brief Default construction, set ID to empty string and location to zero
     * 
     */
    Anchor() {
      ID = "";
      location = Vector3();
    }

    /**
     * @brief Construct a new Anchor object
     * 
     * @param location_
     * @param ID_ 
     */
    Anchor(Vector3 location_, string ID_) {
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

ostream& operator<<(std::ostream &strm, const Anchor &a) {
  return strm << "Anchor(\"" << a.ID << "\", x=" << a.location.x() << ", y=" << a.location.y() << ", z=" << a.location.z() << ")";
}

/**
 * @brief a generic template for both the camera emulator and physical camera
*/
class CameraWrapper {
    private:
        Camera* camera;
    public:
        Camera* getCamera() {return camera;}
        virtual Point2 sample(Point3 tag);
        virtual ~CameraWrapper();
};

/**
 * @brief a generic template for both the sensor emulator and physical sensors
*/
class Sensor {
    public:
        virtual map<pair<Anchor,Anchor>,double> sample(Anchor tag);
        virtual map<pair<Anchor,Anchor>,double> sampleA2a();
};

