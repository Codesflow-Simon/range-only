#include "emulator.h"
#include "logging.h"
#include "factors.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
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
void add_rangeFactors(Graph* graph, Values* values);

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
auto tagNoise =  gtsam::noiseModel::Isotropic::Sigma(3,4);
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.10);

int main() {
  init_anchors();
  emulator = getEmulator();
  tag = Anchor( Vector3(1,-0.5,0.2), "1000"); // Set actual tag location

  init_log();
  write_log(tag.to_string_());

  Graph graph;
  Values values;
  
  add_priors(&graph, &values);
  add_anchors(&graph, &values);
  add_rangeFactors(&graph, &values);

  graph.print();
  values.print();

  Values optimised = values;
  optimised = GaussNewtonOptimizer(graph, optimised).optimize();

  cout << "final tag: " << endl << optimised.at<Point3>(X(0)) << endl;

  close_log();
}

void add_priors(Graph* graph, Values* values) {
  write_log("adding tag priors\n");
  graph->addPrior(X(0), Point3(0,0,0), tagNoise);
  values->insert(X(0), Point3(0,0,0));

  write_log(values->at<Point3>(X(0)));
}

void add_anchors(Graph* graph, Values* values) {
  write_log("adding anchors\n");
  for (int i=0; i<n; i++) {
    write_log("adding anchor" + to_string(i) + "\n" );
    graph->addPrior(L(i), (Point3) anchorMatrix.row(i), anchorNoise);
    values->insert(L(i), (Point3) anchorMatrix.row(i));
  }
}

void add_rangeFactors(Graph* graph, Values* values) {
  map<string,double> sample = emulator.sample(tag);

  for (auto pair : sample) {  // Key-value pair AKA ID-measurment pair
    int index = index_table[pair.first];

    auto factor = RangeFactor<Point3, Point3> (X(0), L(index), pair.second, distNoise);
    // auto factor = DistanceFactor (X(0), L(index), pair.second, distNoise);

    write_log("Adding DistanceFactor " + to_string(index) + " with measurement " + to_string(pair.second) + "\n");
    write_log((Point3)anchorMatrix.row(index));
    write_log("\n");

    graph->add(factor);
  }

}
/**
 * @brief Initialises anchors
 * 
 */
void init_anchors() {
  anchorMatrix = MatrixXd::Zero(n,3);

    anchorMatrix <<
    0,0,0.1,
    1,1,0,
    1,0,1,
    2,0,0,
    1,-1,0,
    1,0,-1;

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

    if (anchorMatrix.size() < n * 3) throw "anchorMatrix bad size";

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
