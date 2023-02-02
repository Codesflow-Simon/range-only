#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include "gtsam/inference/Symbol.h"

#pragma once

using namespace std;

ofstream debugLog;
int matrix = 0;

/**
 * @brief Opens log
 * 
 */
void init_log() {
  debugLog.open("log.txt", ofstream::out | ofstream::trunc);
}

/**
 * @brief Closes log
 * 
 */
void close_log() {
  debugLog << "Closing\n";
  debugLog.close();
}

/**
 * @brief Writes to log
 * 
 * @param str 
 */
void write_log(string str) {
    debugLog << str;
}

/**
 * @brief Writes to log
 * 
 * @param str 
 */
void write_matrix(MatrixXd mat, string id) {
  ofstream matrixLog;
  matrixLog.open(id + ".csv", ofstream::out | ofstream::trunc);
  for (int i=0; i<mat.cols()-1; i++) matrixLog << ",";
  matrixLog << "\n";
  for (int i=0; i<mat.rows(); i++) {
    for (int j=0; j<mat.cols(); j++) {
      matrixLog << mat(i,j);
      if (j != mat.cols()-1) matrixLog << ",";
    }
    matrixLog << endl;
  }
  matrixLog.close();
}

typedef Eigen::Matrix<double,3,1> Vector3d;

 /**
  * Returns key as a human-readable string
 */
string keyToString(Symbol key) {
  return string(1,key.chr()) + to_string(key.index());
}

 /**
  * Returns vector as a human-readable string
 */
string vecToString(Vector3d vec) {
  return "Vector3d( " + to_string(vec(0)) + ", " + to_string(vec(1)) + ", " + to_string(vec(2)) + ")";
}

/**
 * @brief Writes to log
 * 
 * @param vector 
 */
void write_log(Vector3d vector) {
  debugLog <<  vecToString(vector) << endl;
}