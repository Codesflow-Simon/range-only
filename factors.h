#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/slam/BetweenFactor.h>

#include "cameraEmulator.h"
#include "sensorEmulator.h"
#include "logging.h"
#include "kernels.h"

using namespace std;
using namespace gtsam;

typedef NonlinearFactorGraph Graph;
typedef PinholeCamera<Cal3_S2> Camera;


/**********************GRAPH ADDERS**************************/

/**
 * @brief Adds priors of the anchor, tag and cameras
 * @param Graph* graph to write too
 * @param Values* values to write too
 * @param Eigen::MatrixXd* matrix of anchor positions
 * @param SharedNoiseModel anchor noise model
 * @param list<CameraWrapper*> just of cameras
 * @param SharedNoiseModel camera noise model
*/
void add_priors(Graph* graph, Values* values, Eigen::MatrixXd anchors, SharedNoiseModel anchorNoise, 
  list<CameraWrapper*> cameras, SharedNoiseModel cameraNoise, SharedNoiseModel tagPriorNoise ,double anchorError=0.1) {
  write_log("adding priors\n");

  // Anchors
  for (int i=0; i<anchors.rows(); i++) {
    write_log("adding anchor" + to_string(i) + "\n" );
    graph->addPrior(Symbol('l', i), (Point3) (anchors.row(i).transpose() + standard_normal_vector3()*anchorError), anchorNoise);
    values->insert(Symbol('l', i), (Point3) (anchors.row(i).transpose() + standard_normal_vector3()*anchorError));
  }

  // Cameras
  int i = 0;
  for (auto camera : cameras) {
    graph->addPrior(Symbol('c', i), camera->getCamera()->pose(), cameraNoise);
    values->insert(Symbol('c', i++), camera->getCamera()->pose());
  }

  // Tag, assume the GP will fill in the value
  graph->addPrior(Symbol('x', 0), (Point3) (Point3(0,0,0)), tagPriorNoise);
}

/**
 * @brief Adds range factors between sensors to the provided graph
 * @param Graph* graph graph to write too
 * @param Sensor* sensor that makes measurements to write too
 * @param Anchor tag to get location
 * @param map<string,Key> Anchor-key map
 * @param SharedNoiseModel noiseModel
 * @param bool include anchor to anchor (a2a) measurements, this improves accuracy of the location of the anchors, but only need to be done once
*/
void add_rangeFactors(Graph* graph, Sensor* sensor, Anchor tag, map<string,Key> keyTable, SharedNoiseModel distNoise, bool include_a2a=false) {
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

    auto factor = RangeFactor<Point3>(keyA, keyB, pair.second, distNoise);
    graph->add(factor);

    write_log("Added DistanceFactor " + keyToString(keyA) + " and " + keyToString(keyB) + " with measurement " + 
               to_string(pair.second) + "\n Anchor at " + vecToString(anchorPair.second.location) + "\n");
  }
}

/**
 * @brief Adds betweenFactor between most recent tag measurements
 * @param Graph* graph graph to write too
 * @param Key previous tag key
 * @param Key this tag key
 * @param SharedNoiseModel noise mode;
*/
void add_naiveBetweenFactors (Graph* graph, Key prevTag, Key tag, SharedNoiseModel betweenNoise) {
  graph->add(BetweenFactor<Point3>(prevTag, tag, Point3(0,0,0), betweenNoise));
}

/**
 * @brief Adds projection factors between the tag and camera to the provided graph
 * @param Graph* graph graph to write too
 * @param Anchor tag
 * @param Key key of the tag
 * @param SharedNoiseModel noise model
*/
void add_cameraFactors(Graph* graph, list<CameraWrapper*> cameras, Anchor tag, Key tagKey, SharedNoiseModel projNoise) {
  write_log("adding GenericProjectionFactors\n");
  int i=0;
  for (auto camera : cameras) {
    Point2 measurement = camera->sample(tag.location);

    write_log("Camera " + vecToString(camera->getCamera()->pose().translation()));
    write_log("Measured: (" + to_string(measurement.x()) + ", " + to_string(measurement.y()) + ")\n\n");

    auto factor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, projNoise, Symbol('c', i++), tagKey, camera->getParams());
    graph->add(factor);
  }
}
/**
 * @brief Replaces each entry in a matrix with a 3x3 identity matrix scaled the the original value
 * @param Matrix
 * @return Matrix
*/
MatrixXd identify3(MatrixXd in) {
  MatrixXd out = MatrixXd::Zero(in.rows()*3, in.cols()*3);
  for (int i=0; i<in.rows(); i++) {
    for (int j=0; j<in.cols(); j++) {
      double scalar = in(i,j);
      out(3*i,3*j) = scalar;
      out(3*i+1,3*j+1) = scalar;
      out(3*i+2,3*j+2) = scalar;
    }
  }
  return out;
}

/**
 * @brief creates a gaussian process prior
 * @param Matrix cholesky, Cholesky decomposition of inverse covariance
 * @param double coefficient for diagonal of noise model
 * @return JacobianFactor, the factor
*/
JacobianFactor makeGaussianFactor(Eigen::Matrix<double,-1,-1> cholesky, int start=0) {
  FastVector<pair<Key, gtsam::Matrix>> terms(cholesky.rows());
  for (int i=0; i<cholesky.rows(); i++) {
    auto keyMatrix = pair<Key, gtsam::Matrix>();
    keyMatrix.first = Symbol('x', i+start);
    MatrixXd mat(3*(cholesky.rows()),3);
    for (int j=0; j<cholesky.rows(); j++) {
      double value = cholesky(j,i);
      Matrix33 block = value * Matrix33::Identity();

      for (int k=0;k<3;k++) mat.row(3*j+k) = block.row(k);
    }
    keyMatrix.second = mat; 
    terms[i] = keyMatrix;
  }

  auto zero = Vector::Zero(3*cholesky.rows());
  auto factor = JacobianFactor(terms, zero); // No noise (identity), baked into terms
  return factor;
}
/**
 * @brief creates a conditional gaussian on  https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Conditional_distributions.
 * Using wikipedia notation, we are computing P(x2|x1) or P(Frontal|Parents) where frontal is a single Point3.
 * @param int start, X-key index to start on
 * @param VectorXd start, X-key index to start on
 * @param double kernel sigma, kernel parameter, see kernel.h
 * @param double kernel length (for RBF), kernel parameter, see kernel.h
*/
GaussianConditional makeGaussianConditional(int start, VectorXd indicies, double kernel_sigma, double kernel_length) {
  int n = indicies.size()-1;
  FastVector<pair<Key, gtsam::Matrix>> terms(indicies.size());

  MatrixXd covariance = rbfKernel(indicies, kernel_sigma, kernel_length);

  MatrixXd K_1_1 = covariance.block(0,0,n,n);
  VectorXd K_2_1 = covariance.block(0,n,n,1);
  Matrix11 K_2_2 = Matrix11(covariance(n,n));
  double cholesky;

  if (n!=0) {
    Eigen::RowVectorXd conditional_coefs = K_2_1.transpose()* K_1_1.inverse();
    Matrix11 new_covariance = K_2_2 - (conditional_coefs* K_2_1);
    cholesky = sqrt(1/new_covariance(0,0));

    for (int i=0;i<indicies.size()-1; i++) { 
      pair<Key, gtsam::Matrix> out;
      out.first = Symbol('x', i+start);
      
      if (i!=indicies.size()-1) {
        // Parents 
        Matrix11 col = conditional_coefs.col(i);
        Matrix33 mat = identify3(col);
        out.second = -mat * cholesky;
      }
      terms[i] = out;
    }
  } else 
    cholesky = sqrt(1/covariance(0,0));

  pair<Key, gtsam::Matrix> out;
  out.first = Symbol('x', start+indicies.size()-1);
  out.second = Matrix33::Identity() * cholesky;
  terms[indicies.size()-1] = out;

  auto zero = Point3(0,0,0);
  auto factor = GaussianConditional(terms, 1, zero); 
  return factor;
}

/**
 * @brief Creates a gaussian process prior for variables in the X(t) chain. Takes the (upper) cholesky of the inverse covariance matrix. 
 * This matrix should represent the kernel in one dimension, this function copies it into three dimensions.
 * @param Graph* graph to write 
 * @param Values* values to write to
 * @param FactorIndices* remove, add factors to remove
 * @param Eigen::Matrix<double,-1,-1>  (upper) cholesky of the inverse covariance matrix for one dimension.
*/
void add_gaussianFactors(Graph* graph, int start, VectorXd indicies, double sigma, double length) {
  auto factor = makeGaussianConditional(start, indicies, sigma, length);
  Values zeros;
  for (int i=0; i<indicies.size(); i++) {
    zeros.insert(Symbol('x', start+i), Point3(0,0,0));
  }
  auto linearFactor = LinearContainerFactor(factor, zeros);
  graph->add(linearFactor);
}

/**
 * @brief Gets data directly from the simulated tag
 * @param Graph* graph to write to
 * @param Anchor tag, to get location
 * @param Key key of tag
 * @param error, error to introduce to model
 * @param SharedNoiseModel noise model
*/
void add_trueFactors(Graph* graph, Anchor tag, Key tagKey, double error, SharedNoiseModel true_noise) {
  Point3 data = tag.location + standard_normal_vector3() * error;
  graph->addPrior(tagKey, data, true_noise);
}
