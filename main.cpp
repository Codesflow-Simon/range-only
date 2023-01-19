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
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Cholesky>

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;
typedef PinholeCamera<Cal3_S2> Camera;

using symbol_shorthand::X;
using symbol_shorthand::L;
using symbol_shorthand::C;

Anchor tag;

// Model parameters
const int samples = 50;
const double kernelSigma = 3; // High for RBF, low for Brownian
const double kernelLength = 1;
const int numSensors=10;
const int gaussianMaxWidth = 50;
const double zero_threshold = 1E-12;

// Simulation
const double increment_sigma = 0.1;
const double velocity_sigma = 0;
const double tagStart_sigma = 1;
const double anchorStart_sigma = 3;
const double true_error = 0.1;

// Provides a lookup table from sensor-IDs to their GTSAM symbols
map<string,Key> keyTable;

auto anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.11);
auto tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,4);
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.12);
auto projNoise = gtsam::noiseModel::Isotropic::Sigma(2,1);
auto cameraNoise = gtsam::noiseModel::Isotropic::Sigma(6,1.2);
auto true_noise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);

/**
 * @brief Builds a matrix of the position of anchors using random numbers
 * @return Eigen::MatrixXd matrix of the anchors positions
 */
Eigen::MatrixXd init_anchors() {
  Eigen::MatrixXd anchors = MatrixXd::Zero(numSensors,3);
  for (int i=0; i<numSensors; i++) {
    anchors.row(i) = standard_normal_vector3()*anchorStart_sigma;
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

  assert(tag.ID != "");
  assert(anchors.size()>0);

  keyTable[tag.ID] = X(0);

  for (int i=0; i<numSensors; i++)  {
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
  Eigen::MatrixXd anchorMatrix = init_anchors();

  Point3 velocity = standard_normal_vector3() * velocity_sigma; // Sets a constant velocity, this will bias the tag movement every time step
  tag = Anchor( standard_normal_vector3() * tagStart_sigma, "1000"); // Set actual tag location, using tag ID to be 1000

  Sensor* sensor = getSensor(anchorMatrix);
  auto cameras = list<CameraWrapper*>{getCamera(Pose3(Rot3::RzRyRx(0,0,0), Point3(0,0,-20))),
                                      getCamera(Pose3(Rot3::RzRyRx(0,M_PI/2,0), Point3(-20,0,0)))};
  auto kernel = rbfKernel(gaussianMaxWidth+1, kernelSigma, kernelLength); // Sigma scales output, length slows oscillation
  // auto kernel = brownianKernel(gaussianMaxWidth+1, kernelSigma);

  auto cholesky = inverseCholesky(kernel);
  
  write_matrix(kernel, "covariance");
  write_matrix(cholesky, "inverse covariance cholesky");

  ISAM2 isam;
  Graph graph;
  Values values, estimated_values;
  FactorIndices remove;
  
  add_priors(&graph, &values, anchorMatrix, anchorNoise, cameras, cameraNoise, keyTable[tag.ID], tagPriorNoise);
  add_gaussianFactors(&graph, &values, &remove, cholesky);


  Eigen::MatrixXd data(samples,9); // Data to export for analysis

  for (int i=0; i<samples; i++) {
    write_log("loop " + to_string(i) + "\n");

    keyTable[tag.ID] = X(i); // Sets the tag Key for the current index    
    tag.location += standard_normal_vector3()*increment_sigma+ velocity; // Move tag
    write_log("tag: " + tag.to_string_());

    write_log("adding factors\n");
    add_rangeFactors(&graph, sensor, tag, keyTable, distNoise);
    // add_cameraFactors(&graph, &values, cameras, tag, keyTable[tag.ID], projNoise);
    // add_trueFactors(&graph, tag, keyTable[tag.ID], true_error, true_noise);

    write_log("Optimising\n");

    isam.update(graph, values, remove);

    graph.resize(0);
    values.clear();
    estimated_values = isam.calculateEstimate();

    data(i,6) = tag.location.x();
    data(i,7) = tag.location.y();
    data(i,8) = tag.location.z();
  }
  /*--------- END SAMPLE LOOP ---------*/
  
  values = isam.calculateBestEstimate();

  for (int i=0; i<samples; i++) {
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
  auto covariance = isam.marginalCovariance(X(samples-1));
  auto residual = tag.location - values.at<Point3>(X(samples-1));

  cout << "final tag: " << endl << values.at<Point3>(X(samples-1)) << endl; // Final report to console
  cout << "tag: " << endl << tag.location << endl;
  cout << "marginal covariance of final position:" << endl << covariance << endl;  
  close_log();
  delete sensor;
  for (auto camera : cameras) delete camera;
}

