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
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/linear/JacobianFactor.h>
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
const int samples = 20;
const double kernelSigma = 0.1;
const double kernelLength = 1;
const int numSensors=10;
const int gaussianMaxWidth = 20;
const double zero_threshold = 1E-12;

FactorIndex factorIndex;
FactorIndex GPIndex;

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
 * @brief Adds priors of the anchor, tag and cameras
 * @param Graph* graph to write too
 * @param Values* values to write too
 * @param list<CameraWrapper*> just of cameras
 * @param Eigen::MatrixXd* matrix of anchor positions
*/
void add_priors(Graph* graph, Values* values, list<CameraWrapper*> cameras, Eigen::MatrixXd anchors) {
  write_log("adding priors\n");

  // Anchors
  for (int i=0; i<numSensors; i++) {
    write_log("adding anchor" + to_string(i) + "\n" );
    graph->addPrior(L(i), (Point3) (anchors.row(i).transpose() + standard_normal_vector3()*0.1), anchorNoise);
    values->insert(L(i), (Point3) (anchors.row(i).transpose() + standard_normal_vector3()*0.1));
  }

  // Tag
  graph->addPrior(keyTable[tag.ID], Point3(0,0,0), tagPriorNoise);
  values->insert(keyTable[tag.ID], Point3(0,0,0));

  // Cameras
  int i = 0;
  for (auto camera : cameras) {
    graph->addPrior(C(i), camera->getCamera()->pose(), cameraNoise);
    values->insert(C(i++), camera->getCamera()->pose());
  }
}

/**
 * @brief Adds range factors between sensors to the provided graph
 * @param Graph* graph graph to write too
 * @param Sensor* sensor that makes measurements to write too
 * @param bool include anchor to anchor (a2a) measurements, this improves accuracy of the location of the anchors, but only need to be done once
*/
void add_rangeFactors(Graph* graph, Sensor* sensor, bool include_a2a=false) {
  // Samples from sensors
  map<pair<Anchor,Anchor>,double> sample = sensor->sample(tag);

  // A2a measurements
  if (include_a2a) {
    map<pair<Anchor,Anchor>,double> A2asample = sensor->sampleA2a();
    sample.insert(A2asample.begin(), A2asample.end()); 
    // sample.merge(A2asample);
  }
  
  for (auto pair : sample) {  // ID-measurement pair from sample
    std::pair<Anchor,Anchor> anchorPair = pair.first;

    Key keyA = keyTable[anchorPair.first.ID];
    Key keyB = keyTable[anchorPair.second.ID];
    assert(keyA != 0 && keyB != 0);

    auto factor = DistanceFactor (keyA, keyB, pair.second, distNoise);

    write_log("Adding DistanceFactor " + keyToString(keyA) + " and " + keyToString(keyB) + " with measurement " + 
               to_string(pair.second) + "\n Anchor at " + vecToString(anchorPair.second.location) + "\n");

    graph->add(factor);
    factorIndex++;
  }
}

/**
 * @brief Adds projection factors between the tag and camera to the provided graph
 * @param Graph* graph graph to write too
 * @param camera* camera that makes measurements to write too
 * @param Cal3_S2::shared_ptr camera calibration
*/
void add_cameraFactors(Graph* graph, Values* values, list<CameraWrapper*> cameras) {
  write_log("adding GenericProjectionFactors\n");
  for (auto camera : cameras) {
    Point2 measurement = camera->sample(tag.location);

    write_log("Camera " + vecToString(camera->getCamera()->pose().translation()));
    write_log("Measured: (" + to_string(measurement.x()) + ", " + to_string(measurement.y()) + ")\n\n");

    Key tagKey = keyTable[tag.ID];
    auto factor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, projNoise, C(0), tagKey, camera->getParams());
    graph->add(factor);
    factorIndex++;
  }
}

/**
 * @brief Adds betweenFactors in the style of a gaussian process. Nodes will be connected with covariance corresponding to the kernel matrix.
 * This is designed to be done incrementally, so only the factors in row 'subject' will be created.
 * This is also constrained by the 'gaussianMaxWidth' hyperparameter that bounds the size of the covariance matrix which limits factors created and helps speed.
 * @param Graph* graph to write to
 * @param Eigen::Matrix<double,-1,-1> kernel containing covariances
 * @param int subject what row of the matrix need to be written
 * @param Velocity will change the mean of the current entry
*/
void add_gaussianFactors(Graph* graph, Eigen::Matrix<double,-1,-1> kernel, int subject, Point3 velocity = Point3()) {
  LLT<MatrixXd> Cholesky(kernel);
  MatrixXd U = Cholesky.matrixU();

  FastVector<pair<Key, gtsam::Matrix>> terms(subject+1);
  for (int i=0; i<=subject; i++) {
    auto keyMatrix = pair<Key, gtsam::Matrix>();
    keyMatrix.first = X(i);
    MatrixXd mat(3*(subject+1),3);
    for (int j=0; j<=subject; j++) {
      double value = U(j,i);
      Matrix33 block = value * Matrix33::Identity();

      for (int k=0;k<3;k++) mat.row(3*j+k) = block.row(k);
    }
    keyMatrix.second = mat; 
    terms[i] = keyMatrix;
    cout << mat << endl;
  }

  auto zero = Vector(3*(subject+1));
  auto noise = noiseModel::Isotropic::Sigma(3*(subject+1), 0.1);

  auto factor = JacobianFactor<Point3>(terms, zero, noise);
  graph->add(factor);
}

/**
 * @brief Gets data directly from the simulated tag
 * @param Graph* graph to write to
*/
void add_trueFactors(Graph* graph) {
  Point3 data = tag.location + standard_normal_vector3() * true_error;
  graph->addPrior(keyTable[tag.ID], data, true_noise);
}

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
  auto kernel = rbfKernel(gaussianMaxWidth, kernelSigma, kernelLength); // Sigma scales output, length slows oscillation
  // auto kernel = brownianKernel(gaussianMaxWidth, kernelSigma);
  
  write_matrix(kernel, "covariance");


  ISAM2 isam;
  Graph graph;
  Values values, estimated_values;
  FactorIndices remove;
  
  add_priors(&graph, &values, cameras, anchorMatrix);

  Eigen::MatrixXd data(samples,9); // Data to export for analysis

  for (int i=0; i<samples; i++) {
    write_log("loop " + to_string(i) + "\n");

    keyTable[tag.ID] = X(i); // Sets the tag Key for the current index    
    tag.location += standard_normal_vector3()*increment_sigma+ velocity; // Move tag
    write_log("tag: " + tag.to_string_());

    if (i!=0){
      Point3 prev = estimated_values.at<Point3>(X(i-1)) + (Point3)velocity;
      values.insert(X(i), prev); // Initially assuming uniform tag movement
    } 

    write_log("adding factors\n");
    // add_rangeFactors(&graph, sensor);
    // add_cameraFactors(&graph, &values, cameras);
    add_gaussianFactors(&graph, kernel, i, velocity);
    add_trueFactors(&graph);

    write_log("Optimising\n");

    // values = LevenbergMarquardtOptimizer(graph, values).optimize(); // Optimisation step
    isam.update(graph, values);

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

