#include <math.h>
#include <Eigen/Dense>

/**
 * @brief function of the RBF kernel for gaussian processes (https://en.wikipedia.org/wiki/Radial_basis_function_kernel)
 * @param int a, first index
 * @param int b, second index
 * @param double sigma, scales output by sigma^2
 * @param double lengthScale, increase to make smoother
 * @return double covariance
*/
double rbfKernelFunction(int a, int b, double sigma, double lengthScale) {
  return pow(sigma,2) * exp(-pow(abs(a-b),2)/(2*pow(lengthScale,2)));
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the rbfKernelFunction to generate covariances
 * @param int size, will create matrix of dimension size+1, meaning passing gaussianMaxWidth with generate the correct sized matrix
 * @param double sigma, will pass to rbfKernelFunction
 * @param double lengthScale, will pass to rbfKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd rbfKernel(int size, double sigma, double lengthScale) {
  int matSize = size+1; //Including diagonal
  auto mat = Eigen::MatrixXd(matSize,matSize); 
  for (int i=0; i<matSize; i++) {
    for (int j=0; j<matSize; j++) {
      mat(i,j) = rbfKernelFunction(i,j,sigma,lengthScale);
    }
  }
  cout << mat << endl;
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
double brownianKernelFunction(int a, int b, int currentTime, int size, double sigma) {
  return pow(sigma,2) * min(currentTime-size+a, currentTime-size+a);
}

/**
 * @brief will fetch a square matrix of dimension `size + 1` using the brownianKernelFunction to generate covariances
 * @param int size, will create matrix of dimension size+1, meaning passing gaussianMaxWidth with generate the correct sized matrix
 * @param double sigma, will pass to brownianKernelFunction
 * @return Eigen::MatrixXd kernel matrix
*/
Eigen::MatrixXd brownianKernel(int size, int timestep, double sigma) {
  int matSize = size+1; //Including diagonal
  auto mat = Eigen::MatrixXd(matSize,matSize); 
  for (int i=0; i<matSize; i++) {
    for (int j=0; j<matSize; j++) {
      mat(i,j) = brownianKernelFunction(i,j,timestep,size,sigma);
    }
  }
  return mat;
} 