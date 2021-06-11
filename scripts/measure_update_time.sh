#!/bin/bash

#rostopic echo -p /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze

#Input variables: duration, bag name.

rosbag record -o ~/catkin_overlay_ws/bags/measure_cpu_time.bag --duration=60 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze &(

sleep 5

./change_stiffness.sh 600 600 600 100 100 1 0

python python_scripts/wait_for_stiffness.py 600 600 600 100 100 1 0

./change_goal.sh -0.5 0 1.0 1 0 0 0

python python_scripts/wait_for_not_moving.py

./change_stiffness.sh 600 600 50 100 100 1 0

./apply_wrench.sh 0 0 4 0 0 1

sleep 5

./apply_wrench.sh 0 0 0 0 0 0 & rosnode kill $( rosnode list | grep '/record_')

)
