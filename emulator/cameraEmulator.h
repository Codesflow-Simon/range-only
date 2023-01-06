#include <iostream>
#include <Eigen/Dense>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <math.h>
#include <list>
#include <map>
#include <random>
#include <random_tools.h>

using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3_S2> Camera;

class CameraEmulator {
  public:
    Camera camera;

    CameraEmulator() = default;

    CameraEmulator(Camera camera_) {
      camera = camera_;
    }

    Point2 samplePixel(Point3 tag) {
      return camera.project2(tag);
    }
};