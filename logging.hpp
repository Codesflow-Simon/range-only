
#include <iostream>
#include <fstream>
#include <string>

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