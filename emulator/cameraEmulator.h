#include <iostream>
#include <Eigen/Dense>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <math.h>
#include <list>
#include <map>
#include <random>
#include <random_tools.h>

using namespace std;
using namespace gtsam;

class PolarCoordinate {
  public:
    double radius, azimuth, altitude;

    PolarCoordinate() { 
      azimuth = altitude = radius = 0;
    }
    
    PolarCoordinate(Point3 point) {
      radius = point.norm();

      if (radius == 0) {
        azimuth = altitude = 0;
        return;
      }
    
      Point3 normalised = point / point.norm();

      azimuth = atan(normalised.x()/(normalised.y()+1e-9));
      if (normalised.y() < 0) azimuth = M_PI + azimuth;
      
      altitude = asin(normalised.z());
    }

    PolarCoordinate(double radius_, double azimuth_, double altitude_) {
      assert (radius >= 0);
      assert (altitude >= -M_PI/2 && altitude <= M_PI/2);
      radius = radius_;
      azimuth = azimuth_;
      altitude = altitude_;
      if (azimuth > M_PI) 
        azimuth = fmod(azimuth, M_PI);

      if (azimuth < -M_PI) 
        azimuth = -fmod(azimuth, M_PI);
    }

    bool equals (PolarCoordinate other, double tolerance = 1e-7) {
      return (toPoint() - other.toPoint()).norm() < tolerance;
    }

    Point3 toPoint() {
      return Point3(sin(azimuth) * cos(altitude) * radius, cos(azimuth) * cos(altitude) * radius, sin(altitude) * radius);
    }

};

ostream& operator<<(std::ostream &strm, const PolarCoordinate &a) {
  return strm << "PolarCoordinate( radius = " << a.radius << ", azimuth = " << a.azimuth << ", altitude = " << a.altitude <<  ")";
}

class Camera {
  private:
    Pose3 pose;
    double lens_width = M_PI/6;
    double lens_height = M_PI/8;

  public:
    Point2 pointToPixel(Point3 point) {
      PolarCoordinate polar(point);
      double x = 2 * polar.azimuth / lens_width + 0.5;
      double y = 2 * polar.altitude / lens_height + 0.5;
      if (x > 1 || x < 0 || y > 1 || y < 0) return Point2(-1,-1);
      return Point2(x,y);
    }

    Point3 pixelToUnit(Point2 point) {
      PolarCoordinate polar;
      polar.radius = 1;
      polar.azimuth = (point.x() - 0.5)*lens_width/2;
      polar.altitude = (point.y() - 0.5)*lens_height/2;
      return polar.toPoint();
    }
};