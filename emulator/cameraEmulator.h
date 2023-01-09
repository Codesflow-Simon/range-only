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
#include "wrappers.h"

using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3_S2> Camera;

class CameraEmulator : public CameraWrapper {
  private:
    Camera* camera;
    Cal3_S2::shared_ptr params;
    Pose3 pose;
  public:

    CameraEmulator() {
      Point3 position = standard_normal_vector3()*3;
      pose = Pose3(Rot3::AxisAngle((Point3)-position, 0.0), position); // will always face origin

      params = Cal3_S2::shared_ptr(new Cal3_S2(60, 640, 480)); // FOV (deg), width, height
      Camera* camera = new Camera(pose, *params);
    };

    CameraEmulator(Camera* camera_) {
      camera = camera_;
    }

    Point2 sample(Point3 tag) {
      return camera->project(tag);
    }

    ~CameraEmulator() {
      delete camera;
    }
};