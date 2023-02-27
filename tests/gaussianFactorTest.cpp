#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam_unstable/nonlinear/LinearizedFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>

#include "random_tools.h"
#include "graph_adders.h"
#include "kernels.h"

double small = 1E-8;

TEST_CASE("Identify3 a") {
  double n = GENERATE(-10,-M_PI,0,3);

  MatrixXd a = identify3(Matrix11(n));
  MatrixXd b = Matrix33::Identity()*n;
  REQUIRE(a.isApprox(b));
}
TEST_CASE("Identify3 b") {
  double n1 = GENERATE(-M_PI,0,3,12.3432);
  double n2 = GENERATE(-M_PI,0,3,12.3432);
  double n3 = GENERATE(-M_PI,0,3,12.3432);
  double n4 = GENERATE(-M_PI,0,3,12.3432);

  Matrix22 mat;
  mat << n1,n2,n3,n4;
  MatrixXd a = identify3(mat);
  MatrixXd b = Matrix33::Identity()*n1;
  REQUIRE(a.block(0,0,3,3).isApprox(b));
  MatrixXd c = Matrix33::Identity()*n2;
  REQUIRE(a.block(0,3,3,3).isApprox(c));
  MatrixXd d = Matrix33::Identity()*n3;
  REQUIRE(a.block(3,0,3,3).isApprox(d));
  MatrixXd e = Matrix33::Identity()*n4;
  REQUIRE(a.block(3,3,3,3).isApprox(e));
}

TEST_CASE ("GaussianFactor RBF has mean zero", "[GaussianFactor, factors]") {
  int samples = GENERATE(1,5,30);
  double sigma = GENERATE(2,5,10);
  double length = GENERATE(0.1,1,4);
  MatrixXd kernel = rbfKernel(range(0,samples), sigma, length); // Sigma scales output, length slows oscillation
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  VectorValues values;
  for (int i=0; i<samples; i++) values.insert(Symbol('x', i), Point3::Zero());

  // Prior mean should be zero
  REQUIRE(factor.error(values) < small);
}

TEST_CASE ("GaussianFactor RBF with conversion has mean zero", "[GaussianFactor, factors]") {
  int samples = GENERATE(2,10);
  double sigma = GENERATE(1,3,5);
  double length = GENERATE(0.1,1,2);

  MatrixXd kernel = rbfKernel(range(0,samples), sigma, length); // Sigma scales output, length slows oscillation
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  Values zeros; 
  Values values;
  for (int i=0; i<samples; i++) {
    Point3 p = standard_normal_vector3();
    values.insert(Symbol('x', i), p);
    zeros.insert(Symbol('x', i), Point3(0,0,0));
  }

  SharedNoiseModel noise = noiseModel::Isotropic::Sigma(3,1);

  // Native linear graph
  GaussianFactorGraph graph1;
  graph1.add(factor);
  VectorValues MAP1 = graph1.optimize();
  
  // Wrapped as a nonlinear graph
  NonlinearFactorGraph graph2;
  auto linearFactor = LinearContainerFactor(factor, zeros);
  graph2.add(linearFactor);
  // Values MAP2 = LevenbergMarquardtOptimizer(graph2, values).optimize();

  // Re-linearized
  // auto graph3 = graph2.linearize(MAP2);
  // VectorValues MAP3 = graph3->optimize();

  // Prior mean should be zero
  for (int i=0; i<samples; i++) {
    REQUIRE(MAP1.at(Symbol('x',i)).norm()         <  1e-5);
    // REQUIRE(MAP2.at<Point3>(Symbol('x',i)).norm() <  1e-5);
    // REQUIRE(MAP3.at(Symbol('x',i)).norm()         <  1e-5);
  }
  
  auto marginals1 = Marginals(graph1, MAP1);
  // auto marginals2 = Marginals(graph2, MAP2);
  // auto marginals3 = Marginals(*graph3, MAP3);

  for (int i=0; i<samples; i++) {
    REQUIRE(marginals1.marginalCovariance(Symbol('x',i)).isApprox(Matrix33::Identity() * sigma*sigma, 1e-5));
    // REQUIRE(marginals2.marginalCovariance(Symbol('x',i)).isApprox(Matrix33::Identity() * sigma*sigma, 1e-5));
    // REQUIRE(marginals3.marginalCovariance(Symbol('x',i)).isApprox(Matrix33::Identity() * sigma*sigma, 1e-5));
  }
}

TEST_CASE("GaussianFactor nonlinear RBF fits data", "[GaussianFactor, factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = rbfKernel(range(0,7), 1, 1);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  // Using data output [0.5, 0.7, 0.7, 0.8, 0.7]

  NonlinearFactorGraph graph;

  Values zeros; 
  Values values;
  for (int i=0; i<7; i++) {
    Point3 p = standard_normal_vector3()*5;
    values.insert(Symbol('x', i), p);
    zeros.insert(Symbol('x', i), Point3(0,0,0));
  }

  auto noise = noiseModel::Isotropic::Sigma(3,0.1);
  graph.addPrior(Symbol('x', 0), Point3(0.5,0,0), noise);
  graph.addPrior(Symbol('x', 1), Point3(0.7,0,0), noise);
  graph.addPrior(Symbol('x', 2), Point3(0.7,0,0), noise);
  graph.addPrior(Symbol('x', 3), Point3(0.8,0,0), noise);
  graph.addPrior(Symbol('x', 4), Point3(0.7,0,0), noise);

  graph.add(LinearContainerFactor(factor, zeros));

  values = LevenbergMarquardtOptimizer(graph, values).optimize();

  // True values: 
  Vector7 truePosterior;
  truePosterior << 0.33228808, 0.47264458, 0.52414984, 0.54867366, 0.43940891, 0.19410022, 0.03811899;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(values.at<Point3>(Symbol('x', i))[0] - truePosterior[i]) < small);
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

TEST_CASE("GaussianFactor nonlinear RBF fits data multidimensional", "[GaussianFactor, factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = rbfKernel(range(0,7), 2, 0.5);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  NonlinearFactorGraph graph;

  Values zeros; 
  Values values;
  for (int i=0; i<7; i++) {
    Point3 p = standard_normal_vector3()*5;
    values.insert(Symbol('x', i), p);
    zeros.insert(Symbol('x', i), Point3(0,0,0));
  }

  auto noise =  gtsam::noiseModel::Isotropic::Sigmas(Vector3d(0.5,0.7,0.7));
  graph.addPrior(Symbol('x', 0), Point3(1.4,1.6,1.6), noise);
  graph.addPrior(Symbol('x', 1), Point3(2.1,2.3,2.3), noise);
  graph.addPrior(Symbol('x', 2), Point3(1.8,2.0,2.0), noise);
  graph.addPrior(Symbol('x', 3), Point3(0.6,0.8,0.8), noise);
  graph.addPrior(Symbol('x', 4), Point3(-0.4,-0.6,-0.6), noise);

  graph.add(LinearContainerFactor(factor, zeros));

  values = LevenbergMarquardtOptimizer(graph, values).optimize();

  Vector7 truePosterior1, truePosterior2;
  truePosterior1 << 1.33083896e+00,  1.99665326e+00,  1.71077722e+00,  5.72658990e-01, -3.72959872e-01, -5.84049617e-02, -1.45128575e-04;
  truePosterior2 << 1.45086338e+00,  2.08917207e+00,  1.81610588e+00,  7.25951734e-01, -5.25538498e-01, -8.20606141e-02, -2.03901415e-04;
  for (int i=0; i<7; i++) {
    REQUIRE(abs(values.at<Point3>(Symbol('x', i))[0] - truePosterior1[i]) < small);
    REQUIRE(abs(values.at<Point3>(Symbol('x', i))[1] - truePosterior2[i]) < small);
    REQUIRE(abs(values.at<Point3>(Symbol('x', i))[2] - truePosterior2[i]) < small);
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
TEST_CASE("GaussianFactor RBF fits data", "[GaussianFactor, factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = rbfKernel(range(0,7), 1, 1);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  // Using data output [0.5, 0.7, 0.7, 0.8, 0.7]

  GaussianFactorGraph graph;
  auto noise =  gtsam::noiseModel::Isotropic::Sigma(3,1);

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
  auto kernel = rbfKernel(range(0,7), 2, 0.5);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  GaussianFactorGraph graph;
  auto noise =  gtsam::noiseModel::Isotropic::Sigmas(Vector3d(0.5,0.7,0.7));
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
  int samples = GENERATE(5,10,20,30);
  double length = GENERATE(0.1,0.5,1,3,5);
  MatrixXd kernel = brownianKernel(range(0,samples), length);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  VectorValues values;
  for (int i=0; i<samples; i++) values.insert(Symbol('x', i), Point3::Zero());

  // Prior mean should be zero
  REQUIRE(factor.error(values) < small);
}

TEST_CASE("GaussianFactor Brownian fits data", "[GaussianFactor, factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = brownianKernel(range(0,7), 0.1);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  // Using data output [0.5, 0.7, 0.7, 0.8, 0.7]

  GaussianFactorGraph graph;
  auto noise =  gtsam::noiseModel::Isotropic::Sigma(3,0.1);
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

TEST_CASE("GaussianFactor Brownian fits data multidimensional", "[GaussianFactor, factors]") {
  // Comparing results to data from another Gaussian Process, only working in x
  auto kernel = brownianKernel(range(0,7), 2);
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  GaussianFactorGraph graph;
  auto noise =  gtsam::noiseModel::Isotropic::Sigmas(Vector3d(0.5,0.7,0.7));
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

TEST_CASE("GaussianConditional has zero mean", "[GaussianConditional, factors]") {
  // Comparing results with makeGaussianFactor
  int datapoints = GENERATE(5,10,30);
  double sigma  = GENERATE(0.5,1.5);
  double length  = GENERATE(0.5,1,3);

  auto kernel = rbfKernel(range(0,datapoints), sigma, length);
  auto cholesky = inverseCholesky(kernel);
  auto dense = makeGaussianFactor(cholesky);

  GaussianFactorGraph graphDense;
  graphDense.add(dense);

  GaussianFactorGraph graphFactors;
  for (int i=0; i<datapoints; i++) {
    auto factor = makeGaussianConditional(0, range(0,i+1), sigma, length);
    graphFactors.add(factor);
  }

  VectorValues a = graphDense.optimize();
  VectorValues b = graphFactors.optimize();

  // Prior mean should be zero
  for (int i=0; i<datapoints; i++) {
    REQUIRE(a.at(Symbol('x',i)).norm() < 1e-8);
    REQUIRE(b.at(Symbol('x',i)).norm() < 1e-8);
  }
}

TEST_CASE("GaussianConditional is factor of GaussianFactor", "[GaussianFactor, GaussianConditional, factors]") {
  // Comparing results with makeGaussianFactor
  int datapoints = 8;
  auto kernel = rbfKernel(range(0,datapoints), 1, 1);
  auto cholesky = inverseCholesky(kernel);
  auto dense = makeGaussianFactor(cholesky);

  GaussianFactorGraph graphDense;
  graphDense.add(dense);

  GaussianFactorGraph graphFactors;
  for (int i=0; i<datapoints; i++) {
    auto factor = makeGaussianConditional(0, range(0,i+1), 1, 1);
    graphFactors.add(factor);
  }

  SharedNoiseModel noise = noiseModel::Isotropic::Sigma(3,1);

  graphDense.add(Symbol('x', 0), Matrix33::Identity(), Point3(-0.5, 1.4, 0.7));
  graphDense.add(Symbol('x', 1), Matrix33::Identity(), Point3(-0.4, 0.2, -0.5));
  graphDense.add(Symbol('x', 2), Matrix33::Identity(), Point3(1.1, -0.7, 0.6));
  graphDense.add(Symbol('x', 3), Matrix33::Identity(), Point3(-0.4, -0.5, -1.1));
  graphDense.add(Symbol('x', 4), Matrix33::Identity(), Point3(0.5, 0.1, 0.3));

  graphFactors.add(Symbol('x', 0), Matrix33::Identity(), Point3(-0.5, 1.4, 0.7));
  graphFactors.add(Symbol('x', 1), Matrix33::Identity(), Point3(-0.4, 0.2, -0.5));
  graphFactors.add(Symbol('x', 2), Matrix33::Identity(), Point3(1.1, -0.7, 0.6));
  graphFactors.add(Symbol('x', 3), Matrix33::Identity(), Point3(-0.4, -0.5, -1.1));
  graphFactors.add(Symbol('x', 4), Matrix33::Identity(), Point3(0.5, 0.1, 0.3));

  VectorValues a = graphDense.optimize();
  VectorValues b = graphFactors.optimize();

  // Prior mean should be zero
  for (int i=0; i<datapoints; i++) {
    REQUIRE((a.at(Symbol('x',i)) - b.at(Symbol('x',i))).norm() < 1e-8);
  }

  auto marginal1 = Marginals(graphDense, a);
  auto marginal2 = Marginals(graphFactors, b);
  for (int i=0; i<datapoints; i++) {
    REQUIRE(marginal1.marginalCovariance(Symbol('x', i)).isApprox( 
            marginal2.marginalCovariance(Symbol('x', i)) ));
  }
}

TEST_CASE("GaussianConditional takes diagonal band of ", "[GaussianFactor, GaussianConditional, factors]") {
  // Comparing results with makeGaussianFactor
  int datapoints = 8;
  int factorSize = 5;
  double sigma = 1;
  double length = 1;
  auto kernel = rbfKernel(range(0,datapoints), sigma, length);

  // Trim kernel
  for (int i=0; i<datapoints; i++) {
    for (int j=0; j<datapoints; j++) {
      if (abs(i-j) > factorSize) kernel(i,j) = 0;
    }
  }

  auto cholesky = inverseCholesky(kernel);
  auto dense = makeGaussianFactor(cholesky);

  GaussianFactorGraph graphDense;
  graphDense.add(dense);

  GaussianFactorGraph graphFactors;
  for (int i=0; i<datapoints; i++) {
    VectorXd indicies; 
    int startIndex;
    if (i<factorSize) {
      indicies = range(0,i+1);
      startIndex = 0;
    } else  {
      indicies = range(i-factorSize, i+1);
      startIndex = i-factorSize;
    }
    auto factor = makeGaussianConditional(startIndex, indicies, sigma, length);
    graphFactors.add(factor);
  }

  VectorValues a = graphDense.optimize();
  VectorValues b = graphFactors.optimize();

  // Prior mean should be zero
  for (int i=0; i<datapoints; i++) {
    REQUIRE((a.at(Symbol('x',i)) - b.at(Symbol('x',i))).norm() < 1e-8);
  }

  auto marginal1 = Marginals(graphDense, a);
  auto marginal2 = Marginals(graphFactors, b);
  for (int i=0; i<datapoints; i++) {
    REQUIRE(marginal1.marginalCovariance(Symbol('x', i)).isApprox( 
            marginal2.marginalCovariance(Symbol('x', i)) ));
  }

  // Add some observations  
  SharedNoiseModel noise = noiseModel::Isotropic::Sigma(3,1);

  graphDense.add(Symbol('x', 0), Matrix33::Identity(), Point3(-0.7, 1.6, 0.9));
  graphDense.add(Symbol('x', 1), Matrix33::Identity(), Point3(-0.6, 0.4, -0.7));
  graphDense.add(Symbol('x', 2), Matrix33::Identity(), Point3(1.3, -0.9, 0.8));
  graphDense.add(Symbol('x', 3), Matrix33::Identity(), Point3(-0.6, -0.7, -1.4));
  graphDense.add(Symbol('x', 4), Matrix33::Identity(), Point3(0.2, 0.3, 0.5));

  graphFactors.add(Symbol('x', 0), Matrix33::Identity(), Point3(-0.7, 1.6, 0.9));
  graphFactors.add(Symbol('x', 1), Matrix33::Identity(), Point3(-0.6, 0.4, -0.7));
  graphFactors.add(Symbol('x', 2), Matrix33::Identity(), Point3(1.3, -0.9, 0.8));
  graphFactors.add(Symbol('x', 3), Matrix33::Identity(), Point3(-0.6, -0.7, -1.4));
  graphFactors.add(Symbol('x', 4), Matrix33::Identity(), Point3(0.2, 0.3, 0.5));

  a = graphDense.optimize();
  b = graphFactors.optimize();

  // Prior mean should be zero
  for (int i=0; i<datapoints; i++) {
    REQUIRE((a.at(Symbol('x',i)) - b.at(Symbol('x',i))).norm() < 1e-1); // Large error since there is odd behavior near zero
  }

  marginal1 = Marginals(graphDense, a);
  marginal2 = Marginals(graphFactors, b);
  for (int i=0; i<datapoints; i++) {
    REQUIRE(marginal1.marginalCovariance(Symbol('x', i)).isApprox( 
            marginal2.marginalCovariance(Symbol('x', i)), 1e-3));
  }
}