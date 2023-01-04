#include "emulator.h"
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
void add_rangeFactors(Graph* graph, Values* values, Key target);

Emulator emulator;
Anchor tag;
int n=6;
double sampling_error = 0.1;
map<string,int> index_table;

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
auto betweenNoise = gtsam::noiseModel::Isotropic::Sigma(3,0.5);

int main() {
  /***
  * Setup
  ***/
  init_log();

  init_anchors();
  emulator = getEmulator();
  tag = Anchor( standard_normal_vector3()*4, "1000"); // Set actual tag location

  ISAM2 isam;
  Graph graph;
  Values values, current_estimate;
  
  write_log("adding tag priors\n");

  graph.addPrior(X(0), Point3(0,0,0), tagPriorNoise);

  add_anchors(&graph, &values);

  /***
  * Sampling loop
  ***/
  for (int i=0; i<100; i++) {
    write_log("loop " + to_string(i) + "\n");

    /***
     * Update tag location
    ***/

    tag.location += standard_normal_vector3()*0.5;
    if (i==0) {
      values.insert(X(0), Point3(0,0,0));
    }
    else {
      values.insert(X(i), current_estimate.at<Point3>(X(i-1)));
      // values.insert(X(i), values.at<Point3>(X(i-1)));
    }
    write_log("tag: " + tag.to_string_());

    /***
     * Add factors
    ***/

    write_log("adding factors\n");
    add_rangeFactors(&graph, &values, X(i));
    if (i!=0)     
      graph.add(BetweenFactor<Point3>(X(i), X(i-1), Point3(0,0,0), betweenNoise));

    write_log("Optimising\n");
    ISAM2Result result = isam.update(graph, values);
    result = isam.update();
    result = isam.update();
    result = isam.update();
    result = isam.update();

    // result.print();

    current_estimate = isam.calculateEstimate();
    // values = GaussNewtonOptimizer(graph, values).optimize();

    values.clear();
    graph.resize(0);

    cout << "final tag: " << endl << current_estimate.at<Point3>(X(i)) << endl;
    // cout << "final tag: " << endl << values.at<Point3>(X(i)) << endl;
    cout << "tag: " << endl << tag.location << endl;
    cout << "error " << (tag.location - current_estimate.at<Point3>(X(i))).norm() << endl << endl;


  }
  close_log();
}

void add_anchors(Graph* graph, Values* values) {
  write_log("adding anchors\n");
  for (int i=0; i<n; i++) {
    write_log("adding anchor" + to_string(i) + "\n" );
    graph->addPrior(L(i), (Point3) anchorMatrix.row(i), anchorNoise);
    values->insert(L(i), (Point3) anchorMatrix.row(i));
  }
}

void add_rangeFactors(Graph* graph, Values* values, Key target) {
  map<string,double> sample = emulator.sample(tag);

  for (auto pair : sample) {  // Key-value pair AKA ID-measurment pair
    int index = index_table[pair.first];

    auto factor = RangeFactor<Point3, Point3> (target, L(index), pair.second, distNoise);
    // auto factor = DistanceFactor (X(0), L(index), pair.second, distNoise);

    write_log("Adding DistanceFactor " + to_string(index) + " with measurement " + to_string(pair.second) + "\n" + "anchor at ");
    write_log((Point3)anchorMatrix.row(index));

    graph->add(factor);
    write_log("Sucessfully added\n\n");
  }
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

  for (int i=0; i<n; i++)  {
    string id = to_string(i);

    // if (anchorMatrix.size() < n * 3)BetweenFactor throw "anchorMatrix bad size";

    index_table[id] = i;
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
