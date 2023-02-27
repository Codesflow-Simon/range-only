#pragma once

#include <random>
#include <iostream>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

typedef Eigen::Matrix<double,3,1> Vector3d;

using namespace std;

std::default_random_engine gen;
normal_distribution<double> d{0,1}; 
mt19937 otherGen;

/**
 * @brief standard normal generator
*/
double standard_normal_generator() {
  return d(gen);
}

/**
 * @brief Random vector3
*/
Vector3d standard_normal_vector3() {
  Vector3d vec;
  for (int i=0;i<3;i++) vec(i) = standard_normal_generator();
  return vec;
}

/**
 * @brief Euclidean norm
*/
double norm(Vector3d vec) {
  double sq_x = vec.x() * vec.x();
  double sq_y = vec.y() * vec.y();
  double sq_z = vec.z() * vec.z();
  return sqrt(sq_x + sq_y + sq_z);
}


/**
 * @brief Euclidean distance
*/
double distanceBetween(Vector3d a, Vector3d b) {
  Vector3d displacement = a - b;
  return norm(displacement);
}
