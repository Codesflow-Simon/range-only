#include <iostream>
#include <Eigen/Dense>
#include <list>
#include <map>
#include <random>
#include <random_tools.h>
#include "wrappers.h"

using namespace std; 
using namespace Eigen;

typedef Eigen::Matrix<double,3,1> Vector3;

/**
 * @brief Emulates a tag-anchor environment
 * 
 */
class SensorEmulator : public Sensor {
  private:
    list<Anchor> anchors;
    double error = 0.1;
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
    map<pair<Anchor,Anchor>,double> sample(Anchor tag) {
      map< pair<Anchor,Anchor> , double > output;

      for (Anchor anchor : anchors) {
        pair<Anchor,Anchor> anchorTagPair(tag, anchor);

        double distance = (tag.location - anchor.location).norm();
        double noise = error * standard_normal_generator();
        output[anchorTagPair] = distance + noise;
      }
      return output;
    }

  

    /**
     * @brief Samples the system given a tag
     * 
     * @param tag 
     * @return vector<double> 
     */
    map<pair<Anchor,Anchor>,double> sampleA2a() {
      map<pair<Anchor,Anchor>,double> output;
      
      for (auto it = anchors.begin(); it != anchors.end(); ++it) {
          for (auto itr = list<Anchor>::iterator(it); itr != anchors.end(); ++itr) {
            Anchor anchorA = *it;
            Anchor anchorB = *itr;

            pair<Anchor,Anchor> anchorTagPair(anchorA, anchorB);

            double distance = (anchorA.location - anchorB.location).norm();
            double noise = error * standard_normal_generator();
            output[anchorTagPair] = distance + noise;
        }
      }
      return output;
    }
};
