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

#include "logging.h"
#include "kernels.h"

using namespace std;
using namespace gtsam;

typedef NonlinearFactorGraph Graph;
typedef PinholeCamera<Cal3_S2> Camera;

/**
 * @brief !! Deprecated, just use the code in the function !! Adds betweenFactor between most recent tag measurements, creates a Brownian motion model
 * @param Graph* graph graph to write too
 * @param Key previous tag key
 * @param Key this tag key
 * @param SharedNoiseModel noise mode;
*/
void add_naiveBetweenFactors (Graph* graph, Key prevTag, Key tag, SharedNoiseModel betweenNoise) {
  graph->add(BetweenFactor<Point3>(prevTag, tag, Point3(0,0,0), betweenNoise));
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
 * @brief given a Cholesky creates a gaussian process prior over Keys X(start) to X(start+n) where n is rows/columns of Cholesky
 * @param Matrix cholesky, Cholesky decomposition of inverse covariance
 * @param double coefficient for diagonal of noise model
 * @return JacobianFactor, the factor
*/
JacobianFactor makeGaussianCholesky(Eigen::Matrix<double,-1,-1> cholesky, int start=0) {
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

  auto zero = VectorXd::Zero(3*cholesky.rows());
  auto factor = JacobianFactor(terms, zero); // No noise (identity), baked into terms
  return factor;
}

/**
 * @brief given a set of integers and covariance function, connects the variables with a multivariate prior Gaussian Process
 * @paragraph Calling makeGaussianProcess with indicies = {t3,t4,t5,t6} will generate P(t3 t4 t5 t6) ~ N(0, h) where h is the matrix generated by the covariance function
 * This is mathematically equivalent to constructing P(t3) P(t4 | t3) P(t5 | t4 t3) P(t6 | t5 t4 t3) using Gaussian Conditionals
 * @param int start, X-key index to start on
 * @param VectorXd indicies, X-key index to start on
 * @param double kernel sigma, kernel parameter, see kernel.h
 * @param double kernel length (for RBF), kernel parameter, see kernel.h
 * @param string cov, covariance function to use, options: "rbf", "brownian", "linear", "arcsin"
 * @return JacobianFactor, the factor
*/
JacobianFactor makeGaussianProcess(int start, VectorXd indicies, double kernel_sigma, double kernel_length, string cov="rbf") {
  MatrixXd covariance;
  if (cov == "brownian") covariance = brownianKernel(indicies, kernel_sigma);
  if (cov == "linear") covariance = linearKernel(indicies, kernel_sigma, kernel_length);
  if (cov == "arcsin") covariance = arcsinKernel(indicies, kernel_sigma);
  else covariance = rbfKernel(indicies, kernel_sigma, kernel_length);

  MatrixXd cholesky = inverseCholesky(covariance);
  return makeGaussianCholesky(cholesky, start);
}

/**
 * @brief creates a conditional Gaussian expression for X(start + indicies.length() - 1) which is dependant on all X(start + i) where i < indicies.length() - 1.
 * Which is staying, given a set of timestamps, the last timestamp is dependant on all other timestamps via the covariance function provided.
 * 
 * @paragraph
 * https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Conditional_distributions.
 * Using wikipedia notation, we are computing P(x2|x1) or P(Frontal|Parents) where frontal is a single Point3.
 * 
 * In general passing indicies as {X0, X1, ..., X(n-1), Xn} will give a distribution P(Xn | X(n-1) X(n-2) ... X0)
 * 
 * For example passing indicies = { 10.0 , 11.1 , 12.0 , 13.1 , 14.0 } and start = 21 means:
 * X(25) {time at t=14.0} is related via covariance function to other indicies:
 * P(X(25) | X(24) X(23) X(22) X(21)) ~ N(0, h(indicies))
 * where h is the covariance function (returning the covariance matrix)
 * 
 * This can be use to construct factorizations of larger probability density functions:
 * 
 * P(X3 X2 X1) = P(X1) P(X2 | X1) P(X3 | X2 X1)
 *
 * @param int start, X-key index to start on
 * @param VectorXd indicies, X-key index to start on
 * @param double kernel sigma, kernel parameter, see kernel.h
 * @param double kernel length (for RBF), kernel parameter, see kernel.h
 * @param string cov, covariance function to use, options: "rbf", "brownian", "linear", "arcsin"
*/
GaussianConditional makeGaussianConditional(int start, VectorXd indicies, double kernel_sigma, double kernel_length, string cov="rbf") {
  int n = indicies.size()-1;
  assert(n>=0);
  FastVector<pair<Key, gtsam::Matrix>> terms(indicies.size());

  MatrixXd covariance;
  if (cov == "brownian") covariance = brownianKernel(indicies, kernel_sigma);
  if (cov == "linear") covariance = linearKernel(indicies, kernel_sigma, kernel_length);
  if (cov == "arcsin") covariance = arcsinKernel(indicies, kernel_sigma);
  else covariance = rbfKernel(indicies, kernel_sigma, kernel_length);

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
 * @brief A non-linear wrapper for makeGaussianConditional, adds directly to a factor graph.
 * @param int start, passed to makeGaussianConditional
 * @param double covariance sigma
 * @param double second parameter to covariance function
*/
LinearContainerFactor makeGaussianConditionalNonlinear(int start, VectorXd indicies, double sigma, double length, string cov="rbf") {
  auto factor = makeGaussianConditional(start, indicies, sigma, length, cov);

  Values zeros;
  for (int i=0; i<indicies.size(); i++) {
    zeros.insert(Symbol('x', start+i), Point3(0,0,0));
  }
  auto linearFactor = LinearContainerFactor(factor, zeros);
  return linearFactor;
}

/**
 * @brief !! Deprecated !! use makeGaussianConditionalNonlinear !! Use  A non-linear wrapper for makeGaussianConditional, adds directly to a factor graph.
 * @param Graph* graph to write 
 * @param int start, passed to makeGaussianConditional
 * @param double covariance sigma
 * @param double second parameter to covariance function
*/
void add_gaussianFactors(Graph* graph, int start, VectorXd indicies, double sigma, double length, string cov="rbf") {
  graph->add(makeGaussianConditionalNonlinear(start, indicies, sigma, length, cov));
}
