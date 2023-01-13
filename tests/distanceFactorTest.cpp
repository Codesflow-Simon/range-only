#include <catch2/catch_test_macros.hpp>
#include <gtsam/geometry/Pose3.h>
#include "factors.h"

Key keyA = (Key) 0;
Key keyB = (Key) 1;

static SharedNoiseModel model(noiseModel::Unit::Create(1));

double small = 1E-8;

TEST_CASE {
    double measurement = 2.71;

    auto factorA = DistanceFactor(keyA, keyB, measurement, model);
    auto factorB = DistanceFactor(keyA, keyB, measurement, model);

    Point3 x = Point3(1,-1,1.5);
    Point3 x = Point3(-3.2,-0.2,0.8);

    REQUIRE(abs(factorA.evaluateError() - factorB.evaluateError()) < small);
}