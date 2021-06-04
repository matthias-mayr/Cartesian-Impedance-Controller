#!/bin/bash


#rostopic echo -p /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze

#Input variables: duration, bag name.
if [[ $# -ne 2 ]]; then
    echo "Illegal number of parameters!"
    exit 2
fi

rosbag record -o ~/catkin_overlay_ws/bags/$2.bag --duration=$1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze /bh/state/CartesianWrench




