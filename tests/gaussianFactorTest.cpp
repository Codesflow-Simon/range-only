#include <catch2/catch_test_macros.hpp>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>
#include "factors.h"
#include "kernels.h"

double small = 1E-8;

TEST_CASE ("GaussianFactorTest", "[factors]") {
  int samples = 20;
  auto kernel = rbfKernel(samples, 4, 1); // Sigma scales output, length slows oscillation
  auto cholesky = inverseCholesky(kernel);
  auto factor = makeGaussianFactor(cholesky);

  VectorValues values;
  for (int i=0; i<samples; i++) values.insert(Symbol('x', i), Point3::Zero());

  REQUIRE(factor.error(values));

  // auto kernel = brownianKernel(gaussianMaxWidth, kernelSigma);

}