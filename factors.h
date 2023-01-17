#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianDensity.h>

using namespace std;
using namespace gtsam;

// This is identical to RangeFactor
class DistanceFactor: public gtsam::NoiseModelFactorN<Point3, Point3> {
  private:

  double measurement;

  public:
    
  DistanceFactor (Key a, Key b, double meas, SharedNoiseModel model) :
    NoiseModelFactorN<Point3, Point3>(model, a, b), measurement(meas) {}
  
  gtsam::Vector evaluateError(
  const Point3& x, const Point3& y, 
  boost::optional<Eigen::MatrixXd&> H1 = boost::none,  
  boost::optional<Eigen::MatrixXd&> H2 = boost::none) 
  const {  
    double distance;
    distance = (double)(x - y).norm() + 0.0001;

    if (H1)
      *H1 = ((x-y)/distance).transpose();

    if (H2)
      *H2 = ((y-x)/distance).transpose();

    return Vector1(distance - measurement);
  }

  gtsam::Vector evaluateError(
  const Pose3& x_, const Point3& y, 
  boost::optional<Eigen::MatrixXd&> H1 = boost::none,  
  boost::optional<Eigen::MatrixXd&> H2 = boost::none) 
  const {  
    Vector3 x = x_.translation();
    double distance;
    distance = (double)(x - y).norm() + 0.0001;

    if (H1)
      *H1 = ((x-y)/distance).transpose();

    if (H2)
      *H2 = ((y-x)/distance).transpose();

    return Vector1(distance - measurement);
  }

};
