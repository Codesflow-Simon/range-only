# Gaussian Processes in Factor Graphs
A library for expressing Gaussian Process priors in GTSAM.

by Simon Little, CSIRO IDENT:LIT079, email:simonclittle@outlook.com

## Quickstart
### Required software
[cmake](cmake.org), <br>
[GTSAM](https://github.com/borglab/gtsam)
[NLohmann Json] (https://github.com/nlohmann/json)

## Install and build
This project uses CMake to build the binaries, with cmake installed use the following to build the project
`mkdir build <br>
 cd build <br>
 cmake .. <br>
 make`

## Structure
The main Gaussian Process library can linked linking the CMake target `gaussian`:

`
target_link_libraries({cmake target} PUBLIC gaussian)
`
