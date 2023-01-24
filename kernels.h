#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#pragma once

using namespace Eigen;

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
 * @brief function of the RBF kernel for gaussian processes (https://en.wikipedia.org/wiki/Radial_basis_function_kernel)
 * @param int a, first index
 * @param int b, second index
 * @param double sigma, scales output by sigma^2
 * @param double lengthScale, increase to make smoother
 * @return double covariance
*/
double rbfKernelFunction(double a, double b double sigma, double lengthScale) {
  return sigma * sigma * exp(-pow(abs(a-b),2)/(2*lengthScale*lengthScale));
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the rbfKernelFunction to generate covariances
 * @param vector<double> indicies
 * @param double sigma, will pass to rbfKernelFunction
 * @param double lengthScale, will pass to rbfKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd rbfKernel(vector<double> indicies, double sigma, double lengthScale) {
  auto mat = Eigen::MatrixXd(indicies.size(), indicies.size()); 
  int i=0;
  for (auto a : indicies) {
    int j=0;
    for (auto b : indicies){
      mat(i,j++) = rbfKernelFunction(a,b,sigma,lengthScale);
    }
    i++;
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
double brownianKernelFunction(double a, double b double sigma) {
  return sigma * sigma * std::min(a+1, b+1);  // Using 1 indexing to avoid zeros making the matrix singular with zeros in first row
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the brownianKernelFunction to generate covariances
 * @param vector<double> indicies
 * @param double sigma, will pass to brownianKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd brownianKernel(vector<double> indicies, double sigma) {
  auto mat = Eigen::MatrixXd(indicies.size(), indicies.size()); 
  int i=0;
  for (auto a : indicies) {
    int j=0;
    for (auto b : indicies){
      mat(i,j++) = brownianKernelFunction(a,b,sigma,lengthScale);
    }
    i++;
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
double linearKernelFunction(double a, double b double sigma, double zero_sigma=0, double x_int=0) {
  return  zero_sigma * zero_sigma + sigma * sigma * (a-x_int) * (b-x_int);  // Using 1 indexing to avoid zeros making the matrix singular with zeros in first row
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the linearKernelFunction to generate covariances
 * @param vector<double> indicies
 * @param double sigma, will pass to linearKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd linearKernel(vector<double> indicies, double sigma, double zero_sigma=0, double x_int=0) {
  auto mat = Eigen::MatrixXd(indicies.size(), indicies.size()); 
  int i=0;
  for (auto a : indicies) {
    int j=0;
    for (auto b : indicies){
      mat(i,j++) = linearKernelFunction(a,b,sigma,lengthScale);
    }
    i++;
  }
  return mat;
} 
