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

/**
 * @brief a generic template for both the camera emulator and physical camera
*/
// class CameraWrapper {
//   protected:
//     Camera* camera;
//     Cal3_S2::shared_ptr params;

//   public:

//     virtual Camera* getCamera() {return camera;}
//     virtual Cal3_S2::shared_ptr  getParams() {return params;}

//     virtual Point2 sample(Point3 tag);
//     virtual ~CameraWrapper();
// };

class CameraEmulator {
  protected:
    Camera* camera;
    Cal3_S2::shared_ptr params;

  public:
    /**
     * @brief getter for camera
     * @return Camera*
    */
    Camera* getCamera() {return camera;}

    /**
     * @brief getter for Cal3_S2 params
     * @return Cal3_S2::shared_ptr
    */
    Cal3_S2::shared_ptr  getParams() {return params;}

    /**
     * Default constructor, creates a camera at (-10,0,0), Cal3_S2(60, 6400, 4800)
    */
    CameraEmulator() {
      Point3 position = Point3(-10,0,0);
      Pose3 pose = Pose3(Rot3::AxisAngle((Point3)-position, 0.0), position); // will always face origin

      params = Cal3_S2::shared_ptr(new Cal3_S2(60, 6400, 4800)); // FOV (deg), width, height
      camera = new Camera(pose, *params);
    };

    /**
     * @brief Camera constructor, copys an existing camera
    */
    CameraEmulator(Camera* camera_) {
      camera = camera_;
    }

    /**
     * @brief Samples the projection of the tag onto the camera 
    */
    Point2 sample(Point3 tag) {
      return camera->Project(tag);
    }

    virtual ~CameraEmulator() {
      delete camera;
    }
};

typedef CameraEmulator CameraWrapper;