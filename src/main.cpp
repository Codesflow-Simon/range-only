#include "logging.h"
#include "graph_adders.h"
#include "kernels.h"

#include "dataEmulator.h"
#include "realSource.h"
#include "random_tools.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <fstream>
#include <nlohmann/json.hpp>

#include <chrono>

using namespace std::chrono;
using namespace gtsam;
using namespace std;

using Graph = NonlinearFactorGraph;
using Camera = PinholeCamera<Cal3_S2>;
using Time = chrono::time_point<chrono::high_resolution_clock>;
using json = nlohmann::json;
using Noise = boost::shared_ptr<gtsam::noiseModel::Isotropic>;

// Model parameters
json parameters;
json anchors;
json path;

Noise masterAnchorNoise;
Noise anchorNoise;
Noise tagPriorNoise;
Noise distNoise;
Noise betweenNoise;

int main(int argc, char *argv[]) {
  init_log();

  /**
   * Import some data files
  */
  std::ifstream f("../params.json");
  parameters = json::parse(f);

  std::ifstream g("../anchors.json");
  anchors = json::parse(g);

  /**
   * Noise models
  */
  masterAnchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["masterAnchorNoise"]);
  anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["anchorNoise"]);
  tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["tagPriorNoise"]);
  distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,(double)parameters["distNoise"]);
  betweenNoise =  gtsam::noiseModel::Isotropic::Sigma(3,(double)parameters["betweenNoise"]);

  /**
   * Setup data pipeline
  */
  DataSource* source = new Emulator();

  /**
   * Setup numerics
  */
  ISAM2Params params(ISAM2GaussNewtonParams(), 0.50, 50, true);
  ISAM2 isam(params);
  Graph graph; // Something is wrong with graph
  Values values, estimated_values;

  /**
   * Prior factors and initial solution
  */
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
    if (anchor != Point3(0,0,0))
      graph.addPrior(key, anchor, anchorNoise);
    else
      graph.addPrior(key, anchor, masterAnchorNoise);
    j++;
  }

  /**
   * Log for data
  */
  MatrixXd data;
  if ((int)parameters["logging"] > 0) {
    data = MatrixXd((int)parameters["logging"], 7); // Data to export for analysis
  }

  /*-------- Begin SAMPLE LOOP --------*/
  int i=0;
  while (true) {
    source->updateTagKey(Symbol('x', i)); // Keep the table pointing to the the current tag
    source->updateTimeIndex(i); // For emulator only, loops synced

    write_log("loop " + to_string(i) + "\n");
    start = chrono::high_resolution_clock::now();

    write_log("adding factors\n");
    add_rangeFactors(&graph, source, distNoise); // Actual measurements

    // "optimised" gives n-Markov approximation of GP, "naive" gives the full GP, "markov" gives markov (Brownian) model 
    if (parameters["method"] == "optimised" || parameters["method"] == "naive") {
      VectorXd indicies; 
      int startIndex;

      if (parameters["method"] == "optimised") {
          indicies = range(max(0,i-(int)parameters["gaussianMaxWidth"]), i+1);
      } else {
        indicies = range(0,i+1);
        startIndex = 0;
      }
      
      add_gaussianFactors(&graph, startIndex, indicies, parameters["kernelSigma"], parameters["kernelLength"]);
    } else if(parameters["method"] == "markov"){
      if (i>1) add_naiveBetweenFactors(&graph, Symbol('x',i-1), Symbol('x',i), betweenNoise);  
    }
    write_log("Optimising\n");

    graph.print();
    auto results = isam.update(graph, values);

    graph.resize(0);
    values.clear();
    estimated_values = isam.calculateEstimate();

    stop = chrono::high_resolution_clock::now();
    data(i,6) = duration_cast<chrono::microseconds>(stop-start).count();

    i++;
    if (i == (int)parameters["logging"]) break;
  }
  /*--------- END SAMPLE LOOP ---------*/
  
  /**
   * Data reporting
  */
  values = isam.calculateBestEstimate();

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
  delete source;
}

