#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <string>

#include <nlohmann/json.hpp>

#include "util.h"
#include "base.h"
#include "random_tools.h"

using namespace std;
using namespace Eigen;

using json = nlohmann::json;

class Emulator : public Sensor {
    private:
        vector<Anchor> anchors;
        vector<Anchor>::iterator it = anchors.begin();

        double error;
        int a2aSent = -1;
        double time = 0;
        const double dt = 0.05;

        string tagID = "0b05";
        default_random_engine generator;
        normal_distribution<double> dist;

        vector<double> sample(Vector3 tag) {
            int idx = 0;
            vector<double> output = vector<double>(anchors.size());
            for (auto const& anchor : anchors) {
                double noise = dist(generator);
                double distance = distanceBetween(anchor.location, tag);

                output.at(idx) = distance + noise;
                
                idx++;
            }
            return output;
        }

    public:
        void setAnchor(Anchor anchor) {
            anchors.insert(it, anchor);    
            it = anchors.end();
        }

        vector<Anchor> getAnchors(){
            return anchors;
        }

        // Change to String-String?
        map<pair<string,string>,double> sample(Anchor tag) {

        }

        void setMeasurementError(double sigma) {
            dist = normal_distribution<double>(0, sigma);
        }

        json getTagData(Vector3 tag) {
            json output = json();

            auto measurement = sample(tag);
            output["id"] = tagID;
            output["acc"] = {0,0,-9.81};
            output["gyro"] = {0.0, 0.0, 0.0};
            output["mag"] = {0, 0, 21};
            output["ts"] = (double)time;

            json meas = json();
        
            auto ids = vector<string>();
            vector<string>::iterator it = ids.begin();
            for (auto const& anchor: anchors) {
                ids.insert(it, anchor.ID);
                it = ids.end();
            }

            meas["a"] = ids;
            meas["d"] = measurement;      
            output["meas"] = meas;

            return output;
        }
        json getJson() {
            return getJson(Vector3(0,0,0));
        }

        json getJson(Vector3 tag) {

            json output = json();
            if (a2aSent==-1) {
                return getTagData(tag);
            } else {
                Anchor subject = anchors.at(a2aSent);

                auto measurement = sample(subject.location);
                output["id"] = subject.ID;
                output["acc"] = {0,0,-9.81};
                output["gyro"] = {0.0, 0.0, 0.0};
                output["mag"] = {0, 0, 21};
                output["ts"] = time;             
    
                json meas = json();

                auto ids = vector<string>();
                vector<string>::iterator it = ids.begin();
                for (auto const& anchor: anchors) {
                    if (anchor == subject) continue;
                    ids.insert(it, anchor.ID);
                    it = ids.end();
                }

                meas["a"] = ids;
                meas["d"] = removeZeros(measurement);  
                output["meas"] = meas;
            }
            a2aSent--;
            time += dt;
            return output;
        }

        void sendA2a() {
            a2aSent = anchors.size()-1;
        }

        void clearA2a() {
            a2aSent = -1;
        }
};

Vector3 addAnchorNoise(Vector3 anchor, double sigma) {
    return anchor + sigma * standard_normal_vector3();
}