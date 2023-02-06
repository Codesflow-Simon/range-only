#include "emulator.h" 
#include "logging.h"
#include "factors.h"
#include "kernels.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2Params.h>

#include <fstream>
#include <nlohmann/json.hpp>

#include <Eigen/Cholesky>
#include <chrono>

using namespace std::chrono;

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;
typedef PinholeCamera<Cal3_S2> Camera;
typedef chrono::time_point<chrono::high_resolution_clock> Time;
typedef nlohmann::json json;

using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::L;
using symbol_shorthand::C;

// Model parameters
json parameters;

// Provides a lookup table from sensor-IDs to their GTSAM symbols
map<string,Key> keyTable;

auto anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.1);
auto projNoise = gtsam::noiseModel::Isotropic::Sigma(2,0.1);
auto cameraNoise = gtsam::noiseModel::Isotropic::Sigma(6,0.1);
auto betweenNoise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto true_noise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);

/**
 * @brief Get the Emulator and sets anchor position with provided matrix
 * @param Eigen::MatrixXd anchors, list of anchor positions to set
 * @return Sensor* pointer to sensor 
 */
Sensor* getSensor(Eigen::MatrixXd anchors) {
  Sensor* sensor = new Emulator();

  assert(anchors.size()>0);

  for (int i=0; i<parameters["numSensors"]; i++)  {
    string id = to_string(i);

    keyTable[id] = L(i);
    sensor->setAnchor(Anchor(anchors.row(i), id));
  }

  return sensor;
}

/**
 * @brief Get the Emulator with default parameters
 * @return CameraWrapper* 
 */
CameraWrapper* getCamera() {
  return new CameraEmulator();
}
/**
 * @brief Get the Emulator with specific pose parameters
 * @return CameraWrapper*
 */
CameraWrapper* getCamera(Pose3 pose) {
  return new CameraEmulator(pose);
}

int main(int argc, char *argv[]) {
  init_log();
  std::ifstream f("../parameters.json");
  parameters = json::parse(f);
  std::ifstream g("../path.json");
  json path = json::parse(g);

  if (argc == 2) {
    parameters["timesteps"] = stoi(argv[1]);
  }
  assert(parameters["timesteps"] > 0);

  Eigen::MatrixXd anchorMatrix = init_anchors();

  Anchor tag;
  //  point = Point3( stod((string)path[0]["x"]), stod((string)path[0]["y"]), stod((string)path[0]["z"]) );
  Point3 point = Point3(0,0,0);
  tag = Anchor(Point3(point), "1000"); // Set actual tag location, using tag ID to be 1000

  Sensor* sensor = getSensor(anchorMatrix);
  keyTable[tag.ID] = X(0);

  int sampleInterval = (int)parameters["samplesInterval"];

  auto cameras = list<CameraWrapper*>{getCamera(Pose3(Rot3::RzRyRx(0,0     ,0), Point3(0,0,-20))),
                                      getCamera(Pose3(Rot3::RzRyRx(0,M_PI/2,0), Point3(-20,0,0)))};

  // ISAM2Params params(ISAM2GaussNewtonParams(), 0.1, 10, true);
  ISAM2Params params;

  ISAM2 isam(params);
  Graph graph;
  Values values, estimated_values;

  int factorIndex = 0;
  
  add_priors(&graph, &values, anchorMatrix, anchorNoise, cameras, cameraNoise, tagPriorNoise, parameters["anchorError"]);

  for (int i=0; i<(int)parameters["timesteps"] + (int)parameters["gaussianMaxWidth"]; i++) {
    Point3 point = standard_normal_vector3()*parameters["initialization_sigma"];
    values.insert(X(i), point);

    values.insert(V(i), Point3(0,0,0));
  }

  Time start;
  Time stop;

  Eigen::MatrixXd data((int)parameters["timesteps"],10); // Data to export for analysis

  for (int i=0; i<parameters["timesteps"]; i++) {
    write_log("loop " + to_string(i) + "\n");
    start = chrono::high_resolution_clock::now();

    keyTable[tag.ID] = X(i); // Sets the tag Key for the current index    
    // point = Point3( stod((string)path[i]["x"]), stod((string)path[i]["y"]), stod((string)path[i]["z"]) );
    point = tag.location + standard_normal_vector3() * parameters["between_sigma"];

    tag.location = point;
    write_log("tag: " + tag.to_string_());

    if (i%sampleInterval==0 && i!=(int)parameters["timesteps"]-1) {

      write_log("adding factors\n");
      if (i == 0) add_rangeFactors(&graph, sensor, tag, keyTable, distNoise ,true);
      else add_rangeFactors(&graph, sensor, tag, keyTable, distNoise);
      // add_cameraFactors(&graph, cameras, tag, X(i), projNoise);
      // add_trueFactors(&graph, tag, keyTable[tag.ID], parameters["true_error"], true_noise);
    }

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
      if (i>1) add_naiveBetweenFactors(&graph, X(i-1), X(i), betweenNoise);  
    } else if(parameters["method"] == "gpmp2") {
      if (i>0) add_gpmp2Factor(&graph, X(i-1), V(i-1), X(i), V(i), betweenNoise);
    }

    write_log("Optimising\n");

    auto results = isam.update(graph, values);

    graph.resize(0);
    values.clear();
    estimated_values = isam.calculateEstimate();

    data(i,6) = tag.location.x();
    data(i,7) = tag.location.y();
    data(i,8) = tag.location.z();
    stop = chrono::high_resolution_clock::now();
    data(i,9) = duration_cast<chrono::microseconds>(stop-start).count();
  }
  /*--------- END SAMPLE LOOP ---------*/
  
  values = isam.calculateBestEstimate();
  write_matrix(range(0,(int)parameters["gaussianMaxWidth"]+1), "covariance");
  write_matrix(isam.getFactorsUnsafe().linearize(values)->jacobian().first, "jacobian");

  for (int i=0; i<parameters["timesteps"]; i++) {
    Point3 estimate = values.at<Point3>(X(i));
    auto covariance = isam.marginalCovariance(X(i));

    data(i,0) = estimate.x();
    data(i,1) = estimate.y();
    data(i,2) = estimate.z();
    data(i,3) = sqrt(covariance(0,0));
    data(i,4) = sqrt(covariance(1,1));
    data(i,5) = sqrt(covariance(2,2));
  }

  write_matrix(anchorMatrix, "anchors");
  write_matrix(data, "data"); // Writes recorded data to file
  auto covariance = isam.marginalCovariance( X( (int)parameters["timesteps"]-1 ) );
  auto residual = tag.location - values.at<Point3>( X((int)parameters["timesteps"]-1) );

  cout << "final tag: " << endl << values.at<Point3>( X( (int)parameters["timesteps"]-1 ) ) << endl;
  cout << "actual tag: " << endl << tag.location << endl;
  close_log();
  delete sensor;
  for (auto camera : cameras) delete camera;
}

