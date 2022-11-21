#include "emulator.hpp"
#include "logging.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <eigen3/Eigen/Dense>

using namespace gtsam;
using namespace std;

typedef NonlinearFactorGraph Graph;

Emulator getEmulator(); // Function prototype
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

int main() {
  emulator = getEmulator();
  tag = Anchor( Vector3(0.5,0.3,0.2), "0000"); // Set actual tag location

  init_log();
    write_log("tag: ");
    write_log(tag.to_string_());
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

/*! \mainpage template
 *
 * \section intro_sec Introduction
 *
 * This is the introduction.
 *
 * \section install_sec Installation
 *
 * \subsection step1 Step 1: Opening the box
 *
 * etc...
 */