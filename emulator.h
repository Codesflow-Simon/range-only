#include <iostream>
#include <Eigen/Dense>
#include <list>
#include <map>
#include <random>

using namespace std; 
using namespace Eigen;

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
     * @brief Tests anchor equality
     * 
     * @param other 
     * @return true 
     * @return false 
     */
    bool equals(const Anchor& other) const {
      const string otherID = other.ID;
      return otherID == ID;
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

std::default_random_engine gen;
normal_distribution<double> d{0,1}; 
double standard_normal_generator() {
  return d(gen);
}

Vector3 standard_normal_vector3() {
  Vector3 vec;
  for (int i=0;i<3;i++) vec(i) = standard_normal_generator();
  return vec;
}

/**
 * @brief Emulates a tag-anchor environment
 * 
 */
class Emulator {
  private:
    list<Anchor> anchors;
    double error;
    std::default_random_engine gen;
    normal_distribution<double> d{0,1}; 

  public:
    /**
     * @brief Add an Anchor object
     * 
     * @param t 
     */
    void setAnchor(Anchor t) {
      anchors.push_back(t);    
    }

    /**
     * @brief Set the Measurement Error
     * 
     * @param sigma 
     */
    void setMeasurementError(double sigma) {
      error = sigma;
    }

    /**
     * @brief Get the Num Anchors object
     * 
     * @return int 
     */
    int getNumAnchors() {
      return anchors.size();
    }
    
    /**
     * @brief Samples the system given a tag
     * 
     * @param tag 
     * @return vector<double> 
     */
    map<string,double> sample(Anchor tag) {
      // vector<double> output = vector<double>(anchors.size());
      map<string,double> output;

      for (Anchor anchor : anchors) {
        double distance = (tag.location - anchor.location).norm();
        double noise = error * standard_normal_generator();
        output[anchor.ID] = distance + noise;
      }
      return output;
    }
};
