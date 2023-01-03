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
auto distNoise =  gtsam::noiseModel::Isotropic::Sigma(1,0.1);

int main() {
  Graph graph = NonlinearFactorGraph();
  graph.addPrior((Key) 0, Point3(0,0,0), anchorNoise);
  graph.addPrior((Key) 1, Point3(1,1,1), tagNoise);
  graph.add(RangeFactor<Point3,Point3>((Key)0, (Key)1, 2, distNoise));

  Values values;

  values.insert((Key) 0, Point3(0,0,0));
  values.insert((Key) 1, Point3(1,1,1));

  Values optimised = values;
  optimised = GaussNewtonOptimizer(graph, optimised).optimize();

  optimised.print();
  graph.print();
}

/**
 * @brief Initialises anchors
 * 
 */
void init_anchors() {
  anchorMatrix = MatrixXd::Zero(n,3);

    anchorMatrix <<
    0,0,0,
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

void add_anchors(Graph* graph, Values* values) {
  for (int i=0; i<n; i++) {
    write_log("adding anchor" + to_string(i) + "\n" );
    graph->addPrior(L(i), (Point3) anchorMatrix.row(i), anchorNoise);
    values->insert(L(i), (Point3) anchorMatrix.row(i));
  }
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
