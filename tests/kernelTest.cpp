#include <catch2/catch_test_macros.hpp>
#include <math.h>
#include "kernels.h"

double small = 1E-8;

using namespace std;

TEST_CASE ("BrownianKernel test", "[kernels]") {
    int size=11;
    double sigma = 1.3;
    auto kernel = brownianKernel(size, sigma);
    for (int i=0; i<size; i++) {
        for (int j=0; j<11; j++) {
            REQUIRE((kernel(i,j) - min(i,j) * sigma*sigma) < small);
            REQUIRE((kernel(i,j) - brownianKernelFunction(i,j,sigma)) < small);
        }
    }
}

TEST_CASE ("rbfKernel test", "[kernels]") {
    int size=11;
    double sigma = 1.3;
    double length = 1.43;
    auto kernel = rbfKernel(size, sigma, length);
    for (int i=0; i<size; i++) {
        for (int j=0; j<11; j++) {
            REQUIRE((kernel(i,j) - sigma * sigma * exp(-pow(abs(a-b),2)/(2*pow(length,2)))) < small);
            REQUIRE((kernel(i,j) - rbfKernelFunction(i,j,sigma,length)) < small);
        }
    }
}