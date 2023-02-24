#include <nlohmann/json.hpp>
#include <set>
#include <map>

#include "cpp/lib/emulator.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include "sensor.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Point3 (x,y,z)
using Sensor = Emulator;
using Noise = boost::shared_ptr<gtsam::noiseModel::Isotropic>;

Noise masterAnchorNoise;
Noise anchorNoise;
Noise tagPriorNoise;
Noise distNoise;

Eigen::Matrix<double, 5, 3> calibrate(JsonSensor* sensor) {
  // Todo, save calibration
  json data = sensor->sample();
  const int numAnchors = data["meas"]["d"].size();
  const string tagID = data["id"];
  const vector<string> anchorIDs = data["meas"]["a"];

  std::ifstream f("../params.json");
  parameters = json::parse(f);

  std::ifstream g("../anchors.json");
  anchors = json::parse(g);

  masterAnchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["masterAnchorNoise"]);
  anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["anchorNoise"]);
  tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["tagPriorNoise"]);
  distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,(double)parameters["distNoise"]);

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();

  map<string, Key> KeyMap;

  for (json anchor_json : anchors) {
    Point3 anchor = Point3(anchor_json["x"], anchor_json["y"], anchor_json["z"]);

    KeyMap[anchor_json["ID"]] = (Key)anchor_json["key"];
    
    values.insert((Key)anchor_json["key"], anchor);
    if (anchor != Point3(0,0,0)) {
      graph -> add(PriorFactor<Point3>((Key)anchor_json["key"], anchor, anchorNoise));
    } else {
      graph -> add(PriorFactor<Point3>((Key)anchor_json["key"], anchor, masterAnchorNoise));
    }
  }

  sensor.sendA2a();

  for (int i=0; i<(int)parameters["calibration_steps"]; i++) {
    json data = sensor.sample();
    string firstID = data["id"];
    Key firstKey = keyMap[firstID];
    
    for (int j=0; j<(data["meas"]["a"]).size(); j++) {
      string secondID = data["meas"]["a"][j];
      Key secondKey = keyMap[secondID];
      double value = data["meas"]["d"][j];
      
      graph -> add(RangeFactor<Point3>(fir, anchorKey, distance, distance_noise_model));
    }
  }

  LevenbergMarquardtOptimizer optimizer(*graph, values);
  values = optimizer.optimize(); 

  Marginals marginals(*graph, values);
  Eigen::MatrixXd outputMatrix(numAnchors,6);

  int i=0;
  for (json anchor_json : anchors) {
    Point3 point = values.at<Point3>((Key)anchor["Key"]);
    data(i,0) = point.x();
    data(i,1) = point.y();
    data(i,2) = point.z();

    auto cov = marginals.marginalCovariance((Key)anchor["Key"]);
    data(i,3) = sqrt(cov(0,0));
    data(i,4) = sqrt(cov(1,1));
    data(i,5) = sqrt(cov(2,2));
    i++;
  }

  write_matrix(outputMatrix, "calibration");

  return outputMatrix;
}