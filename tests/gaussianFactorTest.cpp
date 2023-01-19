#include <catch2/catch_test_macros.hpp>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include "factors.h"
#include "kernels.h"

double small = 1E-8;

TEST_CASE ("GaussianFactorTest", "[factors]") {
  int samples = 20;
  MatrixXd kernel = rbfKernel(samples, 4, 1); // Sigma scales output, length slows oscillation
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  VectorValues values;
  for (int i=0; i<samples; i++) values.insert(Symbol('x', i), Point3::Zero());

  // Prior mean should be zero
  REQUIRE(factor.error(values) < small);
}

TEST_CASE("GaussianFactorTestCompare", "[factors") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = rbfKernel(7, 1, 1);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  // Using data output [0.5, 0.7, 0.7, 0.8, 0.7]

  GaussianFactorGraph graph;
  SharedDiagonal noise = SharedDiagonal(noiseModel::Diagonal::Sigmas(Vector3(1,1,1)));
  graph.add(Symbol('x', 0), Matrix33::Identity(), Point3(1,0,0) * 0.5, noise);
  graph.add(Symbol('x', 1), Matrix33::Identity(), Point3(1,0,0) * 0.7, noise);
  graph.add(Symbol('x', 2), Matrix33::Identity(), Point3(1,0,0) * 0.7, noise);
  graph.add(Symbol('x', 3), Matrix33::Identity(), Point3(1,0,0) * 0.8, noise);
  graph.add(Symbol('x', 4), Matrix33::Identity(), Point3(1,0,0) * 0.7, noise);

  graph.add(factor);

  VectorValues values;
  values = graph.optimizeDensely();

  // True values: 
  Vector7 truePosterior;
  truePosterior << 0.33228808, 0.47264458, 0.52414984, 0.54867366, 0.43940891, 0.19410022, 0.03811899;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(values.at(Symbol('x', i))[0] - truePosterior[i]) < small);
  }

  auto margin = Marginals(graph, values);
  Vector7 trueVar;
  trueVar << 0.44892178, 0.39556135, 0.39528471, 0.39556135, 0.44892178, 0.81462436, 0.99034773;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(0,0) - trueVar[i]) < small);
  }
  for (int i=0; i<7; i++) {
    REQUIRE_FALSE(abs(margin.marginalCovariance(Symbol('x', i))(0,0) - (trueVar[i]+1)) < small);
  }
}

TEST_CASE("GaussianFactorTestCompare2", "[factors") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = rbfKernel(7, 2, 0.5);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  GaussianFactorGraph graph;
  SharedDiagonal noise = SharedDiagonal(noiseModel::Diagonal::Sigmas(Vector3(0.5,0.7,0.7)));
  graph.add(Symbol('x', 0), Matrix33::Identity(), Point3(1,1,0) * 1.4, noise);
  graph.add(Symbol('x', 1), Matrix33::Identity(), Point3(1,1,0) * 2.1, noise);
  graph.add(Symbol('x', 2), Matrix33::Identity(), Point3(1,1,0) * 1.8, noise);
  graph.add(Symbol('x', 3), Matrix33::Identity(), Point3(1,1,0) * 0.6, noise);
  graph.add(Symbol('x', 4), Matrix33::Identity(), Point3(1,1,0) * -0.4, noise);

  graph.add(factor);

  VectorValues values;
  values = graph.optimizeDensely();

  // True values: 
  Vector7 truePosterior;
  truePosterior << 1.33083896e+00,  1.99665326e+00,  1.71077722e+00,  5.72658990e-01, -3.72959872e-01, -5.84049617e-02, -1.45128575e-04;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(values.at(Symbol('x', i))[0] - truePosterior[i]) < small);
  }

  auto margin = Marginals(graph, values);
  Vector7 trueVar;
  trueVar << 0.23504762, 0.23479699, 0.23479301, 0.23479699, 0.23504762, 3.92993577, 3.99999957;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(0,0) - trueVar[i]) < small);
  }
}