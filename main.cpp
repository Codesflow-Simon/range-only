#include "sensorEmulator.h" 
#include "cameraEmulator.h" 
#include "logging.h"
#include "factors.h"
#include "kernels.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

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
using symbol_shorthand::L;
using symbol_shorthand::C;

// Model parameters
json parameters;

// Provides a lookup table from sensor-IDs to their GTSAM symbols
map<string,Key> keyTable;

auto anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,4);
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.1);
auto projNoise = gtsam::noiseModel::Isotropic::Sigma(2,0.1);
auto cameraNoise = gtsam::noiseModel::Isotropic::Sigma(6,0.1);
auto betweenNoise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto true_noise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);

/**
 * @brief Builds a matrix of the position of anchors using random numbers
 * @return Eigen::MatrixXd matrix of the anchors positions
 */
Eigen::MatrixXd init_anchors() {
  Eigen::MatrixXd anchors = MatrixXd::Zero(parameters["numSensors"],3);
  for (int i=0; i<parameters["numSensors"]; i++) {
    anchors.row(i) = standard_normal_vector3()*parameters["anchorStart_sigma"];
  }
  return anchors;
}

/**
 * @brief Get the Emulator and sets anchor position with provided matrix
 * @param Eigen::MatrixXd anchors, list of anchor positions to set
 * @return Sensor* pointer to sensor 
 */
Sensor* getSensor(Eigen::MatrixXd anchors) {
  SensorEmulator* sensor = new SensorEmulator();

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

int main() {
  init_log();
  std::ifstream f("../parameters.json");
  parameters = json::parse(f);

  Eigen::MatrixXd anchorMatrix = init_anchors();

  Anchor tag;
  tag = Anchor( standard_normal_vector3() * parameters["tagStart_sigma"], "1000"); // Set actual tag location, using tag ID to be 1000

  Sensor* sensor = getSensor(anchorMatrix);
  keyTable[tag.ID] = X(0);

  auto cameras = list<CameraWrapper*>{getCamera(Pose3(Rot3::RzRyRx(0,0     ,0), Point3(0,0,-20))),
                                      getCamera(Pose3(Rot3::RzRyRx(0,M_PI/2,0), Point3(-20,0,0)))};
  auto kernel = rbfKernel(parameters["gaussianMaxWidth"], parameters["kernelSigma"], parameters["kernelLength"]);
  // auto kernel = brownianKernel(parameters["gaussianMaxWidth"], kernelSigma);

  auto cholesky = inverseCholesky(kernel);
  
  write_matrix(kernel, "covariance");
  write_matrix(cholesky, "inverse covariance cholesky");

  ISAM2 isam;
  Graph graph;
  Values values, estimated_values;
  FactorIndices remove;

  int factorIndex;
  int GPIndex;
  
  add_priors(&graph, &values, anchorMatrix, anchorNoise, cameras, cameraNoise, &factorIndex, parameters["anchorError"]);

  GPIndex = factorIndex; 
  add_gaussianFactors(&graph, &values, &remove, cholesky, &factorIndex);

  Time start;
  Time stop;

  Eigen::MatrixXd data((int)parameters["samples"],10); // Data to export for analysis

  for (int i=0; i<parameters["samples"]; i++) {
    write_log("loop " + to_string(i) + "\n");
    start = chrono::high_resolution_clock::now();

    keyTable[tag.ID] = X(i); // Sets the tag Key for the current index    
    tag.location += standard_normal_vector3()*parameters["increment_sigma"]; // Move tag
    write_log("tag: " + tag.to_string_());

    write_log("adding factors\n");
    if (i==0) add_rangeFactors(&graph, sensor, tag, keyTable, distNoise, &factorIndex ,true);
    else add_rangeFactors(&graph, sensor, tag, keyTable, distNoise, &factorIndex);
    add_cameraFactors(&graph, cameras, tag, X(i), projNoise, &factorIndex);

    // if (i>1) add_naiveBetweenFactors(&graph, X(i-1), X(i), betweenNoise, &factorIndex);  
    // add_trueFactors(&graph, tag, keyTable[tag.ID], true_error, true_noise, &factorIndex);

    write_log("Optimising\n");

    auto results = isam.update(graph, values, remove);

    graph.resize(0);
    values.clear();
    estimated_values = isam.calculateEstimate();

    data(i,6) = tag.location.x();
    data(i,7) = tag.location.y();
    data(i,8) = tag.location.z();
    stop = chrono::high_resolution_clock::now();
    data(i,9) = duration_cast<chrono::milliseconds>(stop-start).count();
  }
  /*--------- END SAMPLE LOOP ---------*/
  
  values = isam.calculateBestEstimate();
  write_matrix(isam.getFactorsUnsafe().linearize(values)->jacobian().first, "jacobian");

  for (int i=0; i<parameters["samples"]; i++) {
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
  auto covariance = isam.marginalCovariance( X( (int)parameters["samples"]-1 ) );
  auto residual = tag.location - values.at<Point3>( X((int)parameters["samples"]-1) );

  cout << "final tag: " << endl << values.at<Point3>( X( (int)parameters["samples"]-1 ) ) << endl;
  cout << "tag: " << endl << tag.location << endl;
  close_log();
  delete sensor;
  for (auto camera : cameras) delete camera;
}

