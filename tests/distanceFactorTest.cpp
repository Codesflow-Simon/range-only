#include <catch2/catch_test_macros.hpp>
#include <gtsam/geometry/Pose3.h>
#include "factors.h"

using namespace gtsam;

Key keyA = (Key) 0;
Key keyB = (Key) 1;

static SharedNoiseModel model(noiseModel::Unit::Create(1));

double small = 1E-8;

TEST_CASE ("DistanceFactortest", "[factors]") {
    double measurement = 2.71;

    auto factorA = RangeFactor<Point3>(keyA, keyB, measurement, model);
    auto factorB = RangeFactor<Point3>(keyA, keyB, measurement, model);

    auto factorZero = RangeFactor<Point3>(keyA, keyB, 0, model);

    Point3 x = Point3(1,-1,1.5);
    Point3 y = Point3(-3.2,-0.2,0.8);
    Point3 zero = Point3::Zero();

    REQUIRE(abs(factorA.evaluateError(x,y)(0) - factorB.evaluateError(x,y)(0)) < small);
    REQUIRE(abs(factorZero.evaluateError(x,zero)(0) - x.norm()) < small);
    REQUIRE(abs(factorZero.evaluateError(y,zero)(0) - y.norm()) < small);

    gtsam::Matrix H1, H2;
    factorA.evaluateError(x,y,H1,H2);
    REQUIRE(H1.isApprox(-H2));
}