# Gaussian Processes in Factor Graphs
A library for expressing Gaussian Process priors in GTSAM.

This library is built 

by Simon Little, CSIRO IDENT:LIT079, email:simonclittle@outlook.com

## Quickstart
### Required software
[cmake](cmake.org), <br>
[GTSAM](https://github.com/borglab/gtsam)
[NLohmann Json] (https://github.com/nlohmann/json)

### Install and build
This project uses CMake to build the binaries, with cmake installed use the following to build the project <br>
`mkdir build`<br>
`cd build`<br>
`cmake ..`<br>
`make`

### Structure
The main Gaussian Process library can used by linking the CMake target `gaussian`, this contains functions which will can add gaussian processes to a factor graph. By default the Gaussian Process will use squared-exponential covariance, however, specific covariance functions can be specified. See `makeGaussianConditional` in the documentation (below).
`
target_link_libraries({cmake target} PUBLIC gaussian)
`

This repository also contains code for studying real-world and simulated range sensor system using gaussian processes under the `sensor` directory. 

