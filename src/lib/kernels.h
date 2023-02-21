#include <math.h>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/3rdparty/Eigen/Eigen/Cholesky>

#pragma once

using namespace Eigen;
using namespace std;

/**
 * @brief Returns the covariance given a Cholesky decomposition of the inverse covariance
 * 
 * @param R Covariance matrix
 * @return Eigen::Matrix upper Cholesky of inverse covariance
 */
Eigen::MatrixXd inverseCholesky(Eigen::MatrixXd kernel) {
  LLT<MatrixXd> Cholesky(kernel.inverse());
  MatrixXd U = Cholesky.matrixU();
  return U;
}

/**
 * @brief Generates the range of numbers from a to b, (a<=b), python style
*/
Eigen::VectorXd range(int a, int b) {
  assert(a<=b);
  Eigen::VectorXd out(b-a);
  for (int i=a; i<b; i++) {
    out[i-a] = i;
  }
  return out;
}

/**
 * @brief function of the RBF kernel for gaussian processes (https://en.wikipedia.org/wiki/Radial_basis_function_kernel)
 * @param int a, first index
 * @param int b, second index
 * @param double sigma, scales output by sigma^2
 * @param double lengthScale, increase to make smoother
 * @return double covariance
*/
double rbfKernelFunction(double a, double b, double sigma, double lengthScale) {
  return sigma * sigma * exp(-pow(abs(a-b),2)/(2*lengthScale*lengthScale));
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the rbfKernelFunction to generate covariances
 * @param VectorXd indicies
 * @param double sigma, will pass to rbfKernelFunction
 * @param double lengthScale, will pass to rbfKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd rbfKernel(VectorXd indicies, double sigma, double lengthScale) {
  auto mat = Eigen::MatrixXd(indicies.size(), indicies.size()); 
  for (int i=0; i<indicies.size(); i++) {
    for (int j=0; j<indicies.size(); j++) {
      double a = indicies[i];
      double b = indicies[j];
      mat(i,j) = rbfKernelFunction(a,b,sigma,lengthScale);
    }
  }
  return mat;
} 

/**
 * @brief function of the Brownian motion kernel for gaussian processes (https://math.stackexchange.com/questions/1273437/brownian-motion-and-covariance)
 * @param int a, first index
 * @param int b, second index
 * @param int sigma, scales output by sigma^2
 * @param double lengthScale, increase to make smoother
 * @return double covariance
*/
double brownianKernelFunction(double a, double b, double sigma) {
  return sigma * sigma * std::min(a+1, b+1);  // Using 1 indexing to avoid zeros making the matrix singular with zeros in first row
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the brownianKernelFunction to generate covariances
 * @param VectorXdindicies
 * @param double sigma, will pass to brownianKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd brownianKernel(VectorXd indicies, double sigma) {
  auto mat = Eigen::MatrixXd(indicies.size(), indicies.size()); 
  for (int i=0; i<indicies.size(); i++) {
    for (int j=0; j<indicies.size(); j++) {
      double a = indicies[i];
      double b = indicies[j];
      mat(i,j) = brownianKernelFunction(a,b,sigma);
    }
  }
  return mat;
} 

/**
 * @brief function of the Brownian motion kernel for gaussian processes (https://math.stackexchange.com/questions/1273437/brownian-motion-and-covariance)
 * @param int a, first index
 * @param int b, second index
 * @param int sigma, scales output by sigma^2
 * @param double lengthScale, increase to make smoother
 * @return double covariance
*/
double linearKernelFunction(double a, double b, double sigma, double zero_sigma=0.1, double x_int=0) {
  return  zero_sigma * zero_sigma + sigma * sigma * (a-x_int) * (b-x_int);  // Using 1 indexing to avoid zeros making the matrix singular with zeros in first row
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the linearKernelFunction to generate covariances
 * @param VectorXd indicies
 * @param double sigma, will pass to linearKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd linearKernel(VectorXd indicies, double sigma, double zero_sigma=0.1, double x_int=0) {
  auto mat = Eigen::MatrixXd(indicies.size(), indicies.size()); 
  for (int i=0; i<indicies.size(); i++) {
    for (int j=0; j<indicies.size(); j++) {
      double a = indicies[i];
      double b = indicies[j];
      mat(i,j) = linearKernelFunction(a,b,sigma,zero_sigma, x_int);
    }
  }
  return mat;
} 

/**
 * @brief function of the arcsin motion kernel for gaussian processes (https://math.stackexchange.com/questions/1273437/brownian-motion-and-covariance)
 * @param int a, first index
 * @param int b, second index
 * @param int sigma, scales output by sigma^2
 * @param double lengthScale, increase to make smoother
 * @return double covariance
*/
double arcsinKernelFunction(double a, double b, double sigma, double zero_sigma=0, double x_int=0) {
  double variance = sigma * sigma;
  return  2 / M_PI * asin((2 * linearKernelFunction(a,b,sigma)) / sqrt((1 + linearKernelFunction(a,a,sigma)) * (1 + linearKernelFunction(b,b,sigma))));  // Using 1 indexing to avoid zeros making the matrix singular with zeros in first row
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the arcsinKernelFunction to generate covariances
 * @param VectorXd indicies
 * @param double sigma, will pass to arcsinKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd arcsinKernel(VectorXd indicies, double sigma, double zero_sigma=0, double x_int=0) {
  auto mat = Eigen::MatrixXd(indicies.size(), indicies.size()); 
  for (int i=0; i<indicies.size(); i++) {
    for (int j=0; j<indicies.size(); j++) {
      double a = indicies[i];
      double b = indicies[j];
      mat(i,j) = arcsinKernelFunction(a+1,b+1,sigma,zero_sigma, x_int);
    }
  }
  return mat;
} 
