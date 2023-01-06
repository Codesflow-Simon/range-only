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
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;

using symbol_shorthand::X;
using symbol_shorthand::L;

/**
 * Emulator setup
 */
Emulator getEmulator(); 
void init_anchors();
void add_anchors(Graph* graph, Values* values);
void add_priors(Graph* graph, Values* values);
void add_rangeFactors(Graph* graph);

Emulator emulator;
Anchor tag;
int n=6;
double sampling_error = 0.1;
map<string,Key> keyTable;

/**
 * The following graph indexing conventions will be used
 * 0: tag
 * 1-n: anchors
 * (n number of anchors)
*/

Eigen::MatrixXd anchorMatrix;

auto anchorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
auto tagPriorNoise =  gtsam::noiseModel::Isotropic::Sigma(3,4);
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.10);
auto betweenNoise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);

int main() {
  init_log();
  init_anchors();

  tag = Anchor( standard_normal_vector3()*1, "1000"); // Set actual tag location
  emulator = getEmulator();

  ISAM2 isam;
  Graph graph;
  Values values, current_estimate;
  
  write_log("adding tag priors\n");
  graph.addPrior(X(0), Point3(0,0,0), tagPriorNoise);

  add_anchors(&graph, &values);

  /***
  * Sampling loop
  ***/
  for (int i=0; i<3; i++) {
    keyTable[tag.ID] = X(i);

    write_log("loop " + to_string(i) + "\n");
    tag.location += standard_normal_vector3()*0.1;

    if (i==0) {
      values.insert(X(0), Point3(0,0,0));
    }
    else {
      values.insert(X(i), current_estimate.at<Point3>(X(i-1)));
    }
    write_log("tag: " + tag.to_string_());

    write_log("adding factors\n");
    add_rangeFactors(&graph);
    if (i!=0)     
      graph.add(BetweenFactor<Point3>(X(i), X(i-1), Point3(0,0,0), betweenNoise));

    write_log("Optimising\n");
    
    ISAM2Result result = isam.update(graph, values);
    result = isam.update();
    result = isam.update();
    result = isam.update();
    result = isam.update();
    result = isam.update();
    result = isam.update();

    current_estimate = isam.calculateEstimate();

    values.clear();
    graph.resize(0);

    cout << "final tag: " << endl << current_estimate.at<Point3>(X(i)) << endl;
    cout << "tag: " << endl << tag.location << endl;
    cout << "marginal covariance of final position:" << endl << isam.marginalCovariance(X(i)) << endl;  
    cout << "error " << (tag.location - current_estimate.at<Point3>(X(i))).norm() << endl << endl;
  }
  close_log();
}

/**
 * @brief Adds anchor priors to graph and value structure
 * @param Graph* graph to write too
 * @param Values* values to write too
*/
void add_anchors(Graph* graph, Values* values) {
  write_log("adding anchors\n");
  for (int i=0; i<n; i++) {
    write_log("adding anchor" + to_string(i) + "\n" );
    
    graph->addPrior(L(i), (Point3) (anchorMatrix.row(i).transpose() + standard_normal_vector3()*0.1), anchorNoise);
    values->insert(L(i), (Point3) (anchorMatrix.row(i).transpose() + standard_normal_vector3()*0.1));
  }
}

/**
 * @brief Adds range factors between sensors to the provided graph
 * @param Graph* graph graph to write too
*/
void add_rangeFactors(Graph* graph) {
  map<pair<Anchor,Anchor>,double> sample = emulator.sample(tag);
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

    write_log("Adding DistanceFactor " + to_string(keyA) + " and " + to_string(keyB) + " with measurement " + to_string(pair.second) + "\n Anchor at ");
    write_log(anchorPair.second.location);
    write_log("\n");

    graph->add(factor);
    write_log("Sucessfully added\n\n");
  }
  /***
   * A2a measurments
  */
}
/**
 * @brief Initialises anchors
 * 
 */
void init_anchors() {
  anchorMatrix = MatrixXd::Zero(n,3);
  for (int i=0; i<n; i++) {
    anchorMatrix.row(i) = standard_normal_vector3() * 3;
  }
}

/**
 * @brief Get the Emulator with preset parameters
 * 
 * @return Emulator 
 */
Emulator getEmulator() {
  Emulator emulator = Emulator();
  emulator.setMeasurementError(sampling_error);

  assert(tag.ID != "");

  keyTable[tag.ID] = X(0);

  for (int i=0; i<n; i++)  {
    string id = to_string(i);

    // if (anchorMatrix.size() < n * 3)BetweenFactor throw "anchorMatrix bad size";

    keyTable[id] = L(i);
    emulator.setAnchor(Anchor(anchorMatrix.row(i), id));
  }

  return emulator;
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
