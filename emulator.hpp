#include <iostream>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <random>

using namespace std; 
using namespace Eigen;

typedef Matrix<double,3,1> Vector3;

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
            return "Anchor(\"" + ID + "\", x=" + to_string(location.x()) + ", y=" + to_string(location.y()) + ", z=" + to_string(location.z()) + ")";
        }
};

ostream& operator<<(std::ostream &strm, const Anchor &a) {
    return strm << "Anchor(\"" << a.ID << "\", x=" << a.location.x() << ", y=" << a.location.y() << ", z=" << a.location.z() << ")";
}

/**
 * @brief Emulates a tag-anchor environment
 * 
 */
class Emulator {
    private:
        list<Anchor> anchors;
        double error;
        default_random_engine generator;
        normal_distribution<double> d{0,1}; 

    public:
        /**
         * @brief Add an Anchor object
         * 
         * @param t 
         */
        void setAnchor(Anchor t) {
            anchors.push_front(t);    
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
         * @brief Samples the system given a tag
         * 
         * @param tag 
         * @return vector<double> 
         */
        vector<double> sample(Anchor tag) {
            vector<double> output = vector<double>(anchors.size());

            int i=0;
            for (Anchor anchor : anchors) {
                double distance = (tag.location - anchor.location).norm();
                double noise = error * d(generator);
                output.at(i++) = distance + noise;
            }
            return output;
        }
};
