# Gaussian Processes in Factor Graphs
A library for expressing Gaussian Process priors in GTSAM.

###  Simon Little IDENT:LIT079 , simonclittle@outlook.com

## Quickstart
### Required software
[cmake](cmake.org), <br>
[GTSAM](https://github.com/borglab/gtsam)
[Nlohmann Json] (https://github.com/nlohmann/json)

## Install and build
This project uses CMake to build the binaries, with cmake installed use the following to build the project
`mkdir build
 cd build
 cmake ..
 make`

## Structure
The main Gaussian Process library can linked linking the CMake target `gaussian`:

`
target_link_libraries({cmake target} PUBLIC gaussian)
`