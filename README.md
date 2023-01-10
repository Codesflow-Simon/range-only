# Install guide
## Required software
[cmake](cmake.org), {break}
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