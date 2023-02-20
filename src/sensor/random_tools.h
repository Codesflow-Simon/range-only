#pragma once

#include <random>
#include <iostream>
#include <Eigen/Dense>

using namespace std;

std::default_random_engine gen;
normal_distribution<double> d{0,1}; 
mt19937 otherGen;
double standard_normal_generator() {
  return d(gen);
}

typedef Eigen::Matrix<double,3,1> Vector3;

Vector3 standard_normal_vector3() {
  Vector3 vec;
  for (int i=0;i<3;i++) vec(i) = standard_normal_generator();
  return vec;
}

double norm(Vector3 vec) {
  double sq_x = vec.x() * vec.x();
  double sq_y = vec.y() * vec.y();
  double sq_z = vec.z() * vec.z();
  return sqrt(sq_x + sq_y + sq_z);
}

double distanceBetween(Vector3 a, Vector3 b) {
  Vector3 displacement = a - b;
  return norm(displacement);
}
