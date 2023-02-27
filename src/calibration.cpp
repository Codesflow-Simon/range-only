#include <nlohmann/json.hpp>
#include <set>
#include <map>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Point3.h>

#include "base.h"
#include "dataEmulator.h"
#include "realSource.h"
#include "logging.h"
#include "random_tools.h"
#include "graph_adders.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; 
using Noise = boost::shared_ptr<gtsam::noiseModel::Isotropic>;

Noise masterAnchorNoise;
Noise anchorNoise;
Noise tagPriorNoise;
Noise distNoise;
Noise betweenNoise;

void calibrate(DataSource* source) {
  // Todo, save calibration
  json data = source->getJson();
  const int numAnchors = (data["meas"]["d"]).size();
  const string tagID = data["id"];
  const vector<string> anchorIDs = data["meas"]["a"];

  std::ifstream f("../params.json");
  json parameters = json::parse(f);

  std::ifstream g("../anchors.json");
  json anchors = json::parse(g);

  masterAnchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["masterAnchorNoise"]);
  anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["anchorNoise"]);
  tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["tagPriorNoise"]);
  distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,(double)parameters["distNoise"]);
  betweenNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["betweenNoise"]);

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();

  graph -> add(PriorFactor<Point3>(X(0), Point3(0,0,0), tagPriorNoise));

  map<string, Key> keyMap;

  for (json anchor_json : anchors) {
    Point3 anchor = Point3(anchor_json["x"], anchor_json["y"], anchor_json["z"]);

    keyMap[anchor_json["ID"]] = (Key)anchor_json["key"];
    string IDDebug = anchor_json["ID"];
    int keyDebug = anchor_json["key"];
    
    if (anchor != Point3(0,0,0)) {
      anchor += standard_normal_vector3()*parameters["anchorNoise"];
      values.insert((Key)anchor_json["key"], anchor);
      graph -> add(PriorFactor<Point3>((Key)anchor_json["key"], anchor, anchorNoise));
    } else {
      anchor += standard_normal_vector3() * (double)parameters["masterAnchorNoise"];
      values.insert((Key)anchor_json["key"], anchor);
      graph -> add(PriorFactor<Point3>((Key)anchor_json["key"], Point3(0,0,0), masterAnchorNoise));
    }
  }

  source->sendA2a();
  ISAM2 isam;
  for (int i=0; i<(int)parameters["calibration_steps"]; i++) {

    source->updateTagKey(Symbol('x', i)); 
    source->updateTimeIndex(i); 

    values.insert(X(i), Point3(0,0,0));
    // if (i>0) add_naiveBetweenFactors(graph, Symbol('x',i-1), Symbol('x',i), betweenNoise);  
    int before = max(0, i-10);
    add_gaussianFactors(graph, before, range(before,i+1), 1, 2);

    auto data = source->sample();
    for (auto entry : data ) {
      pair<Key,Key> keys = entry.first;
      double dist = entry.second;
      
      graph -> add(RangeFactor<Point3>(keys.first, keys.second, dist, distNoise));
    }
  }

  // graph->print();
  LevenbergMarquardtOptimizer optimizer(*graph, values);
  values = optimizer.optimize();
  Marginals marginals(*graph, values);
  Eigen::MatrixXd outputMatrix(numAnchors,6);

  int i=0;
  for (json anchor_json : anchors) {
    Point3 point = values.at<Point3>((Key)anchor_json["key"]);
    outputMatrix(i,0) = point.x();
    outputMatrix(i,1) = point.y();
    outputMatrix(i,2) = point.z();

    auto cov = marginals.marginalCovariance((Key)anchor_json["key"]);
    outputMatrix(i,3) = sqrt(cov(0,0));
    outputMatrix(i,4) = sqrt(cov(1,1));
    outputMatrix(i,5) = sqrt(cov(2,2));
    i++;
  }

  write_matrix(outputMatrix, "calibration");
}

int main(int argc, char *argv[]) {
  init_log();

  standard_normal_vector3();

  DataSource* dataSource = new Emulator();  

  calibrate(dataSource);
}