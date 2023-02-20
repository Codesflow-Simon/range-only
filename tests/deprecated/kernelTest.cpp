#include <catch2/catch_test_macros.hpp>
#include <math.h>
#include <iostream>
#include "kernels.h"

double small = 1E-8;

using namespace std;

TEST_CASE ("BrownianKernel test", "[kernels]") {
    int size=11;
    double sigma = 1.3;
    auto kernel = brownianKernel(range(0,size), sigma);
    for (int i=0; i<size; i++) {
        for (int j=0; j<11; j++) {
            REQUIRE((kernel(i,j) - min(i+1,j+1) * sigma*sigma) < small);
            REQUIRE((kernel(i,j) - brownianKernelFunction(i,j,sigma)) < small);
        }
    }
}

TEST_CASE ("rbfKernel test", "[kernels]") {
    int size=11;
    double sigma = 1.3;
    double length = 1.43;
    auto kernel = rbfKernel(range(0,size), sigma, length);
    for (int i=0; i<size; i++) {
        for (int j=0; j<11; j++) {
            REQUIRE((kernel(i,j) - sigma * sigma * exp(-pow(abs(i-j),2)/(2*pow(length,2)))) < small);
            REQUIRE((kernel(i,j) - rbfKernelFunction(i,j,sigma,length)) < small);
        }
    }

    auto kernel5 = rbfKernel(range(0,5),1,1);

    Eigen::Matrix<double,5,5> mat;
    mat <<
    1.00000000e+00, 6.06530660e-01, 1.35335283e-01, 1.11089965e-02, 3.35462628e-04,
    6.06530660e-01, 1.00000000e+00, 6.06530660e-01, 1.35335283e-01, 1.11089965e-02,
    1.35335283e-01, 6.06530660e-01, 1.00000000e+00, 6.06530660e-01, 1.35335283e-01,
    1.11089965e-02, 1.35335283e-01, 6.06530660e-01, 1.00000000e+00, 6.06530660e-01,
    3.35462628e-04, 1.11089965e-02, 1.35335283e-01, 6.06530660e-01, 1.00000000e+00;

    REQUIRE(mat.isApprox(kernel5, 1e-6));
}