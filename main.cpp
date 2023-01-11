#include "sensorEmulator.h" 
#include "cameraEmulator.h" 
#include "logging.h"
#include "factors.h"

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Cal3_S2.h>

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;
typedef PinholeCamera<Cal3_S2> Camera;

using symbol_shorthand::X;
using symbol_shorthand::L;
using symbol_shorthand::C;

Anchor tag;
int n=20;
map<string,Key> keyTable;

/**
 * The following graph indexing conventions will be used
 * 0: tag
 * 1-n: anchors
 * (n number of anchors)
*/

auto anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.2);
auto tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,10);
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.1);

auto covariance = Eigen::Matrix<double, 3, 3>:: Identity();
auto betweenNoise = gtsam::noiseModel::Gaussian::Covariance(0.1*covariance);
// auto betweenNoise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto projNoise = gtsam::noiseModel::Isotropic::Sigma(2,1);
auto cameraNoise = gtsam::noiseModel::Isotropic::Sigma(6,1);

/**
 * @brief Adds anchor priors to graph and value structure
 * @param Graph* graph to write too
 * @param Values* values to write too
*/
void add_priors(Graph* graph, Values* values, CameraWrapper* camera, Eigen::MatrixXd anchors) {
  write_log("adding priors\n");
  for (int i=0; i<n; i++) {
    write_log("adding anchor" + to_string(i) + "\n" );
    
    graph->addPrior(L(i), (Point3) (anchors.row(i).transpose() + standard_normal_vector3()*0.1), anchorNoise);
    values->insert(L(i), (Point3) (anchors.row(i).transpose() + standard_normal_vector3()*0.1));
  }

  values->insert(X(0), Point3(0,0,0));
  graph->addPrior(C(0), camera->getCamera()->pose(), cameraNoise);
  values->insert(C(0), camera->getCamera()->pose());
}

/**
 * @brief Adds range factors between sensors to the provided graph
 * @param Graph* graph graph to write too
 * @param Sensor* sensor that makes measurements to write too
*/
void add_rangeFactors(Graph* graph, Sensor* sensor) {
  map<pair<Anchor,Anchor>,double> sample = sensor->sample(tag);
  // map<pair<Anchor,Anchor>,double> A2asample = emulator.sampleA2a();
  // sample.merge(A2asample);
  
  /***
   * Tag measurments
  */
  for (auto pair : sample) {  // Key-value pair AKA ID-measurment pair
    std::pair<Anchor,Anchor> anchorPair = pair.first;

    Key keyA = keyTable[anchorPair.first.ID];
    Key keyB = keyTable[anchorPair.second.ID];

    assert(keyA != 0 && keyB != 0);

    auto factor = DistanceFactor (keyA, keyB, pair.second, distNoise);

    write_log("Adding DistanceFactor " + keyToString(keyA) + " and " + keyToString(keyB) + " with measurement " + to_string(pair.second) + "\n Anchor at ");
    write_log(anchorPair.second.location);
    write_log("\n");

    graph->add(factor);
    write_log("Sucessfully added\n\n");
  }

}

/**
 * @brief Adds projection factors between the tag and camera to the provided graph
 * @param Graph* graph graph to write too
 * @param camera* camera that makes measurements to write too
 * @param Cal3_S2::shared_ptr camera calibration
*/
void add_cameraFactors(Graph* graph, Values* values, CameraWrapper* camera) {
  write_log("adding projection factors between " + keyToString(C(0)) + " and " + keyToString(X(0)) + "\n");
  Point2 measurement = camera->sample(tag.location);
  write_log("Measured: (" + to_string(measurement.x()) + ", " + to_string(measurement.y()) + ")\n\n");
  Key tagKey = keyTable[tag.ID];
  auto factor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, projNoise, C(0), tagKey, camera->getParams());
  graph->add(factor);
}


/**
 * @brief Initialises anchors
 * @return Eigen::MatrixXd matrix of the anchors positions
 */
Eigen::MatrixXd init_anchors() {
  Eigen::MatrixXd anchors = MatrixXd::Zero(n,3);
  for (int i=0; i<n; i++) {
    anchors.row(i) = standard_normal_vector3() * 3;
  }
  return anchors;
}

/**
 * @brief Get the Emulator with preset parameters
 * 
 * @return Emulator 
 */
Sensor* getSensor(Eigen::MatrixXd anchors) {
  SensorEmulator* sensor = new SensorEmulator();

  assert(tag.ID != "");
  assert(anchors.size()>0);

  keyTable[tag.ID] = X(0);

  for (int i=0; i<n; i++)  {
    string id = to_string(i);

    keyTable[id] = L(i);
    sensor->setAnchor(Anchor(anchors.row(i), id));
  }

  return sensor;
}

/**
 * @brief Get the Emulator with preset parameters
 * 
 * @return Emulator 
 */
CameraWrapper* getCamera() {
  return new CameraEmulator();
}

int main() {
  init_log();
  Eigen::MatrixXd anchorMatrix = init_anchors();

  Vector3 velocity = standard_normal_vector3() * 0.0; 

  tag = Anchor( standard_normal_vector3()*1, "1000"); // Set actual tag location

  Sensor* sensor = getSensor(anchorMatrix);

  CameraWrapper* camera = getCamera();

  Graph graph;
  Values values, current_estimate;
  
  write_log("adding tag priors\n");
  graph.addPrior(X(0), Point3(0,0,0), tagPriorNoise);

  add_priors(&graph, &values, camera, anchorMatrix);

  /***
  * Sampling loop
  ***/
  for (int i=0; i<100; i++) {
    keyTable[tag.ID] = X(i);

    write_log("loop " + to_string(i) + "\n");
    tag.location += standard_normal_vector3()*0.1 + velocity;

    if (i!=0){
      values.insert(X(i), values.at<Point3>(X(i-1)));
    }
    write_log("tag: " + tag.to_string_());

    write_log("adding factors\n");
    // add_rangeFactors(&graph, sensor);
    // add_cameraFactors(&graph, &values, camera);
    if (i!=0)     
      graph.add(BetweenFactor<Point3>(X(i-1), X(i), velocity, betweenNoise)); // Between factor uses X(i-1) + velocity ~= X(i)

    write_log("Optimising\n");
    
    values = LevenbergMarquardtOptimizer(graph, values).optimize();

    write_matrix(graph.linearize(values)->jacobian().first, "jacobian");
  }
  auto covariance = Marginals(graph, values).marginalCovariance(X(99));
  auto residual = tag.location - values.at<Point3>(X(99));

  cout << "final tag: " << endl << values.at<Point3>(X(99)) << endl;
  cout << "tag: " << endl << tag.location << endl;
  cout << "marginal covariance of final position:" << endl << covariance << endl;  
  cout << "Mahalanobis " << sqrt(residual.transpose() * covariance.inverse() * residual) << endl << endl;
  close_log();
  delete sensor;
  delete camera;
}

// Doxygen mainpage

/* 
 * To build and run
 * \code{.sh} 
 * mkdir build
 * cd build
 * cmake ..
 * make
 * ./main
 * \endcode 
 * 
 * Generate documentation
 * \code{.sh}
 * doxygen docs_conf
 * \endcode 
 *
 * etc...
 */
