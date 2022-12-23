#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

using namespace std;

ofstream debugLog;

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

typedef Eigen::Matrix<double,3,1> Vector3;

/**
 * @brief Writes to log
 * 
 * @param vector 
 */
void write_log(Vector3 vector) {
    debugLog <<  "Vector3( "<< vector(0) << ", " << vector(1) << ", " << vector(2) << ")" << endl;
}