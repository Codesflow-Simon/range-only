#include <catch2/catch_test_macros.hpp>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include "factors.h"
#include "kernels.h"

double small = 1E-8;

TEST_CASE ("GaussianFactorTest RBF", "[factors]") {
  int samples = 20;
  MatrixXd kernel = rbfKernel(samples, 4, 1); // Sigma scales output, length slows oscillation
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  VectorValues values;
  for (int i=0; i<samples; i++) values.insert(Symbol('x', i), Point3::Zero());

  // Prior mean should be zero
  REQUIRE(factor.error(values) < small);
}

TEST_CASE ("GaussianFactorTest ZeroMeanPrior RBF", "[factors]") {
  int samples = 20;
  MatrixXd kernel = rbfKernel(samples, 4, 1); // Sigma scales output, length slows oscillation
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  VectorValues values;
  for (int i=0; i<samples; i++) values.insert(Symbol('x', i), standard_normal_vector3()*5);

  NonlinearFactorGraph
  // Prior mean should be zero
  REQUIRE(factor.error(values) < small);
}

TEST_CASE("GaussianFactorTestCompare RBF", "[factors]") {
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

TEST_CASE("GaussianFactorTestCompare2 RBF", "[factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = rbfKernel(7, 2, 0.5);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  GaussianFactorGraph graph;
  SharedDiagonal noise = SharedDiagonal(noiseModel::Diagonal::Sigmas(Vector3(0.5,0.7,0.7)));
  graph.add(Symbol('x', 0), Matrix33::Identity(), Point3(1.4,1.6,1.6), noise);
  graph.add(Symbol('x', 1), Matrix33::Identity(), Point3(2.1,2.3,2.3), noise);
  graph.add(Symbol('x', 2), Matrix33::Identity(), Point3(1.8,2.0,2.0), noise);
  graph.add(Symbol('x', 3), Matrix33::Identity(), Point3(0.6,0.8,0.8), noise);
  graph.add(Symbol('x', 4), Matrix33::Identity(), Point3(-0.4,-0.6,-0.6), noise);

  graph.add(factor);

  VectorValues values;
  values = graph.optimizeDensely();

  Vector7 truePosterior1, truePosterior2;
  truePosterior1 << 1.33083896e+00,  1.99665326e+00,  1.71077722e+00,  5.72658990e-01, -3.72959872e-01, -5.84049617e-02, -1.45128575e-04;
  truePosterior2 << 1.45086338e+00,  2.08917207e+00,  1.81610588e+00,  7.25951734e-01, -5.25538498e-01, -8.20606141e-02, -2.03901415e-04;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(values.at(Symbol('x', i))[0] - truePosterior1[i]) < small);
    REQUIRE(abs(values.at(Symbol('x', i))[1] - truePosterior2[i]) < small);
    REQUIRE(abs(values.at(Symbol('x', i))[2] - truePosterior2[i]) < small);
  }

  auto margin = Marginals(graph, values);
  Vector7 trueVar1, trueVar2;
  trueVar1 << 0.23504762, 0.23479699, 0.23479301, 0.23479699, 0.23504762, 3.92993577, 3.99999957;
  trueVar2 << 0.43572534, 0.4349131,  0.4349016,  0.4349131,  0.43572534, 3.93379569, 3.99999959;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(0,0) - trueVar1[i]) < small);
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(1,1) - trueVar2[i]) < small);
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(2,2) - trueVar2[i]) < small);
  }
}

TEST_CASE ("GaussianFactorTest Brownian", "[factors]") {
  int samples = 20;
  MatrixXd kernel = brownianKernel(samples, 1);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  VectorValues values;
  for (int i=0; i<samples; i++) values.insert(Symbol('x', i), Point3::Zero());

  // Prior mean should be zero
  REQUIRE(factor.error(values) < small);
}

TEST_CASE("GaussianFactorTestCompare Brownian", "[factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = brownianKernel(7, 0.1);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  // Using data output [0.5, 0.7, 0.7, 0.8, 0.7]

  GaussianFactorGraph graph;
  SharedDiagonal noise = SharedDiagonal(noiseModel::Diagonal::Sigmas(Vector3(1,1,1)*0.1));
  graph.add(Symbol('x', 0), Matrix33::Identity(), Point3(1,0,0) * 0.5, noise);
  graph.add(Symbol('x', 1), Matrix33::Identity(), Point3(1,0,0) * 0.7, noise);
  graph.add(Symbol('x', 2), Matrix33::Identity(), Point3(1,0,0) * 0.7, noise);
  graph.add(Symbol('x', 3), Matrix33::Identity(), Point3(1,0,0) * 0.8, noise);
  graph.add(Symbol('x', 4), Matrix33::Identity(), Point3(1,0,0) * 0.7, noise);

  graph.add(factor);

  VectorValues values;
  values = graph.optimizeDensely();

  Vector7 truePosterior;
  truePosterior << 0.35842697, 0.5752809, 0.66741573, 0.72696629, 0.71348315, 0.71348315, 0.71348315;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(values.at(Symbol('x', i))[0] - truePosterior[i]) < small);
  }

  auto margin = Marginals(graph, values);
  Vector7 trueVar;

  trueVar << 0.00382022, 0.00438202, 0.00449438, 0.0047191, 0.00617978, 0.01617978, 0.02617978;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(0,0) - trueVar[i]) < small);
  }
  for (int i=0; i<7; i++) {
    REQUIRE_FALSE(abs(margin.marginalCovariance(Symbol('x', i))(0,0) - (trueVar[i]+1)) < small);
  }
}

TEST_CASE("GaussianFactorTestCompare2 Brownian", "[factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = brownianKernel(7, 2);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  GaussianFactorGraph graph;
  SharedDiagonal noise = SharedDiagonal(noiseModel::Diagonal::Sigmas(Vector3(0.5,0.7,0.7)));
  graph.add(Symbol('x', 0), Matrix33::Identity(), Point3(1.4,1.6,1.6), noise);
  graph.add(Symbol('x', 1), Matrix33::Identity(), Point3(2.1,2.3,2.3), noise);
  graph.add(Symbol('x', 2), Matrix33::Identity(), Point3(1.8,2.0,2.0), noise);
  graph.add(Symbol('x', 3), Matrix33::Identity(), Point3(0.6,0.8,0.8), noise);
  graph.add(Symbol('x', 4), Matrix33::Identity(), Point3(-0.4,-0.6,-0.6), noise);

  graph.add(factor);

  VectorValues values;
  values = graph.optimizeDensely();

  Vector7 truePosterior1, truePosterior2;
  truePosterior1 << 1.35773145, 2.03916611, 1.74725848, 0.61148654, -0.34050079, -0.34050079, -0.34050079;
  truePosterior2 << 1.49981045, 2.18174707, 1.89835363, 0.78519391, -0.44883184, -0.44883184, -0.44883184;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(values.at(Symbol('x', i))[0] - truePosterior1[i]) < small);
    REQUIRE(abs(values.at(Symbol('x', i))[1] - truePosterior2[i]) < small);
    REQUIRE(abs(values.at(Symbol('x', i))[2] - truePosterior2[i]) < small);
  }

  auto margin = Marginals(graph, values);
  Vector7 trueVar1, trueVar2;
  trueVar1 << 0.22291236, 0.22360464, 0.22360691, 0.2236455,  0.23606798, 4.23606798, 8.23606798;
  trueVar2 << 0.39746022, 0.40138455, 0.40142714, 0.40181746, 0.44131112, 4.44131112, 8.44131112;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(0,0) - trueVar1[i]) < small);
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(1,1) - trueVar2[i]) < small);
    REQUIRE(abs(margin.marginalCovariance(Symbol('x', i))(2,2) - trueVar2[i]) < small);
  }
}