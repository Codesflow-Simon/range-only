#include <random>
#include <iostream>
#include <Eigen/Dense>

#pragma once

using namespace std;

std::default_random_engine gen;
normal_distribution<double> d{0,1}; 
double standard_normal_generator() {
  return d(gen);
}

typedef Eigen::Matrix<double,3,1> Vector3;

Vector3 standard_normal_vector3() {
  Vector3 vec;
  for (int i=0;i<3;i++) vec(i) = standard_normal_generator();
  return vec;
}