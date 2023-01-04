#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

using namespace std;

ofstream debugLog;
int matrix = 0;

/**
 * @brief Opens log
 * 
 */
void init_log() {
    debugLog.open("log", ofstream::out | ofstream::trunc);
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
    for (int i=0; i<mat.rows(); i++) {
        for (int j=0; j<mat.cols(); j++) {
            matrixLog << mat(i,j);
            if (j != mat.cols()-1) matrixLog << ",";
        }
        matrixLog << endl;
    }
    matrixLog.close();
}

typedef Eigen::Matrix<double,3,1> Vector3;

/**
 * @brief Writes to log
 * 
 * @param vector 
 */
void write_log(Vector3 vector) {
    debugLog <<  "Vector3( "<< vector(0) << ", " << vector(1) << ", " << vector(2) << ")" << endl;
}
