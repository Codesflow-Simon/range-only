#include "logging.h"
#include "factors.h"
#include "kernels.h"

#include "dataEmulator.h"
#include "realSource.h"
#include "random_tools.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <fstream>
#include <nlohmann/json.hpp>

#include <chrono>

using namespace std::chrono;

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;
typedef PinholeCamera<Cal3_S2> Camera;
typedef chrono::time_point<chrono::high_resolution_clock> Time;
typedef nlohmann::json json;

// Model parameters
json parameters;
json anchors;
json path;

auto anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.01);

auto betweenNoise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto true_noise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);

int main(int argc, char *argv[]) {
  init_log();

  std::ifstream f("../params.json");
  parameters = json::parse(f);

  std::ifstream g("../anchors.json");
  anchors = json::parse(g);

  RealSource* dataSource = new RealSource(parameters["source"]);
  JsonSensor* sensor = new JsonSensor(dataSource);

  ISAM2Params params(ISAM2GaussNewtonParams(), 0.50, 50, true);
  ISAM2 isam(params);
  Graph graph; // Something is wrong with graph
  Values values, estimated_values;

  list<CameraWrapper*> cameras;

  graph.addPrior(Symbol('x', 0), (Point3) (Point3(0,0,0)), tagPriorNoise);
  
  // Need to happen Dynamically
  for (int i=0; i<1000; i++) {
    Point3 point = standard_normal_vector3()*parameters["initialization_sigma"];
    values.insert(Symbol('x',i), point);
  }

  Time start;
  Time stop;

  int j=0;
  for (json anchor_json : anchors) {
    unsigned long int key = anchor_json["key"];
    Point3 anchor = Point3(anchor_json["x"], anchor_json["y"], anchor_json["z"]);
    values.insert(key, anchor);
    graph.addPrior(key, anchor, anchorNoise);
    j++;
  }

  // Expand dynamically
  MatrixXd data((int)parameters["logging"], 7); // Data to export for analysis

  int i=0;
  while (true) {
    sensor->updateTagKey(Symbol('x', i));
    dataSource->updateTimeIndex(i);

    write_log("loop " + to_string(i) + "\n");
    start = chrono::high_resolution_clock::now();

    write_log("adding factors\n");
    add_rangeFactors(&graph, sensor, distNoise);

    if (parameters["method"] == "optimised" || parameters["method"] == "naive") {
      VectorXd indicies; 
      int startIndex;

      if (parameters["method"] == "optimised") {
        if (i<(int)parameters["gaussianMaxWidth"]) {
          indicies = range(0,i+1);
          startIndex = 0;
        } else  {
          indicies = range(i-(int)parameters["gaussianMaxWidth"], i+1);
          startIndex = i-(int)parameters["gaussianMaxWidth"];
        }
      } else {
        indicies = range(0,i+1);
        startIndex = 0;
      }
      
      add_gaussianFactors(&graph, startIndex, indicies, parameters["kernelSigma"], parameters["kernelLength"]);
    } else if(parameters["method"] == "gtsam"){
      if (i>1) add_naiveBetweenFactors(&graph, Symbol('x',i-1), Symbol('x',i), betweenNoise);  
    }
    write_log("Optimising\n");

    auto results = isam.update(graph, values);

    // graph.print();
    graph.resize(0);
    values.clear();
    estimated_values = isam.calculateEstimate();

    stop = chrono::high_resolution_clock::now();
    data(i,6) = duration_cast<chrono::microseconds>(stop-start).count();

    i++;
    if (i == (int)parameters["logging"]) break;
  }
  /*--------- END SAMPLE LOOP ---------*/
  
  values = isam.calculateBestEstimate();
  write_matrix(range(0,(int)parameters["gaussianMaxWidth"]+1), "covariance");

  for (int i=0; i<(int)parameters["logging"]; i++) {
    Point3 estimate = values.at<Point3>(Symbol('x',i));
    auto covariance = isam.marginalCovariance(Symbol('x',i));

    data(i,0) = estimate.x();
    data(i,1) = estimate.y();
    data(i,2) = estimate.z();
    data(i,3) = sqrt(covariance(0,0));
    data(i,4) = sqrt(covariance(1,1));
    data(i,5) = sqrt(covariance(2,2));
  }
  write_matrix(data, "data"); // Writes recorded data to file

  close_log();
  delete sensor;
  for (auto camera : cameras) delete camera;
}

