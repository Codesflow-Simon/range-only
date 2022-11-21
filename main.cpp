#include "emulator.hpp"
#include "logging.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <eigen3/Eigen/Dense>

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <eigen3/Eigen/Dense>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;

Emulator getEmulator(); 
Emulator emulator;
Anchor tag;
#define n 10;

Eigen::Matrix<double,n,3> anchorMatrix <<
  1, 0, 0,
  0, 1, 0,
  0, 0, 1,
 -1, 0, 0,
  0,-1, 0,
  0, 0,-1,
 -1, 1, 0,
  1, 0,-1,
  0,-1, 1,
  2,-1, 1;

/**
 * The following indexing conventions will be used
 * 0: tag
 * 1-n: anchors
 * (n number of anchors)
*/

Eigen::Matrix<double,10,3> anchorMatrix <<
  1, 0, 0,
  0, 1, 0,
  0, 0, 1,
 -1, 0, 0,
  0,-1, 0,
  0, 0,-1,
 -1, 1, 0,
  1, 0,-1,
  0,-1, 1,
  2,-1, 1;

/**
 * The following indexing conventions will be used
 * 0: tag
 * 1-n: anchors
 * (n number of anchors)
*/

Eigen::Matrix<double,10,3> anchorMatrix <<
  1, 0, 0,
  0, 1, 0,
  0, 0, 1,
 -1, 0, 0,
  0,-1, 0,
  0, 0,-1,
 -1, 1, 0,
  1, 0,-1,
  0,-1, 1,
  2,-1, 1;

int main() {
    emulator = getEmulator();
    tag = Anchor( Vector3(0.5,0.3,0.2), "0000"); // Set actual tag location

    init_log();
    write_log(tag.to_string_());
  close_log();

  Graph graph;
  Values values;

  graph.addPrior((Key) 0, Vector3(0,0,0), eigen::Matrix33);
  add_anchors(graph);

}

void add_anchors(Graph graph) {
  graph.addPrior();
  close_log();

  Graph graph;
  
}

void add_anchors(Graph graph) {
  for (int i=1; i<n; i++)
    graph.addPrior((Key) i, (Vector3) anchorMatrix.row(i-1), noiseModel::Isotropic(3,10));
}


/**
 * @brief Get the Emulator with preset parameters
 * 
 * @return Emulator 
 */
Emulator getEmulator() {
  Emulator emulator = Emulator();
  emulator.setMeasurementError(0.1);

  for (int i=1; i<n+1; i++)
    emulator.setAnchor(Anchor(anchorMatrix.row(i-1)), "00"+10));

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
