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
An executable `main` should be in the build directory

## Build documentation
This project uses Doxygen to generate documentation
`
doxygen doxygen.conf
`