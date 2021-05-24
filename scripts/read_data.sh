#!/bin/bash

data=$(rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/position)

#position
x=$(echo ${data} | awk '{print $2}')
echo "x= $x"

y=$(echo ${data} | awk '{print $4}')
echo "y= $y"

z=$(echo ${data} | awk '{print $6}')
echo "z= $z"

diff=$((x-y)) 


echo $diff
