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

  emulator.setAnchor(Anchor(Vector3(-0.6, 0.9,  2.1), "2e4f"));
  emulator.setAnchor(Anchor(Vector3( 1.2, 0.1,  0.3), "3bfc"));
  emulator.setAnchor(Anchor(Vector3(-0.3, 0.3, -1.6), "3a85"));
  emulator.setAnchor(Anchor(Vector3( 0.7, 2.1, -0.4), "3dd2"));
  emulator.setAnchor(Anchor(Vector3(-1.8, 1.5,  0.1), "2e01"));
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