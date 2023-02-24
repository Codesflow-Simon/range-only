#  Gaussian process and distance measurement in GTSAM
##  Simon Little IDENT:LIT079 , simonclittle@outlook.com

# Install guide
## Required software
[cmake](cmake.org), <br>
[GTSAM](https://github.com/borglab/gtsam)


## Install and build
This project uses CMake to build the binaries
`
git clone {this repository}
mkdir build
cd build
cmake ..
make
`
The executables `main` and `jsonDump` should be in the build directory

## Build documentation
This project uses Doxygen to generate documentation
`
doxygen
`

# The program
## System
The system contains one sensor denoted "tag", and several other sensors denoted "anchors". All sensors can make distance measurements to other sensors. The anchor sensors are assumed to be stationary while the tag follows a Gaussian random walk. To model this system, a factor graph is created. This factor graph contains two classes of factors. 

The tag motion is modelled with a Gaussian Process. This utilizes a custom factor to set covariances between timesteps.

The distance measurements between the tag and anchors factors that constrain the motion of the particle.

## Repository
The source code in the `src` directory contains two main sub-folders

