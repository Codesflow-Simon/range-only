#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

using namespace std;
using namespace gtsam;

// typedef Eigen::Matrix<double,3,1> Vector3;

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

      return Vector1(output);
  }

  // virtual ~DistanceFactor() {}
};