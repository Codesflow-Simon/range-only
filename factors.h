#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace std;
using namespace gtsam;

typedef Eigen::Matrix<double,3,1> Vector3;

class DistanceFactor: public gtsam::NoiseModelFactor2<Vector3, Vector3> {
  private:

  double measurement;
  Key k;

  public:
    
  DistanceFactor (Key a, Key b, double meas, SharedNoiseModel model) :
    NoiseModelFactor2<Vector3, Vector3>(model, a, b), measurement(meas), k(b) {}
  
  gtsam::Vector evaluateError(
  const X1& a, const X2& b, 
  boost::optional<Eigen::MatrixXd&> H1 = boost::none,  
  boost::optional<Eigen::MatrixXd&> H2 = boost::none) 
  const {  
      double distance;
      distance = (double)(a - b).norm() + 0.0001;

      if (H1)
        *H1 = ((a-b)/distance).transpose();

      if (H2)
        *H2 = ((b-a)/distance).transpose();

      gtsam::Vector output(1);
      output(0,0) = distance;
      return output;
  }
};