#!/bin/bash

cd build
make 

tests=( 10 20 30 40 50 60 70 80 90 100 125 150 175 200 )
dataLocation="../data/testGtsam"
mkdir $dataLocation

for steps in "${tests[@]}"; do
    ./main $steps
    mv ./anchors.csv $dataLocation/anchors$steps.csv 
    mv ./data.csv $dataLocation/data$steps.csv 
    mv ./jacobian.csv $dataLocation/jacobian$steps.csv 
    mv ./covariance.csv $dataLocation/covariance$steps.csv 
done
