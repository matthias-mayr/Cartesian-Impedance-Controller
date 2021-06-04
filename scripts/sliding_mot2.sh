#!/bin/bash

#Sliding motion
#Table : z: 0.707

./home_position_sliding.sh
echo "Press any key to start"
read a
force=10
time_simulation=200
./log_data.sh $time_simulation sliding_motion2_force$force &(

./change_stiffness.sh 50 1000 1 200 200 1 0
python python_scripts/wait_for_stiffness.py 1000 1000 1 200 200 1 0
sleep 5
./apply_wrench.sh 0 0 $force 0 0 0
python python_scripts/wait_for_wrench.py 0 0 $force 0 0 0
python python_scripts/wait_for_not_moving.py

x=-0.531824529171
y=-0.421
./change_goal.sh -0.7 $y 0.71 1 0 0 0
sleep 2
python python_scripts/wait_for_not_moving.py

./apply_wrench.sh 0 0 0 0 0 0
python python_scripts/wait_for_wrench.py 0 0 0 0 0 0

sleep 2
)
rosnode kill $( rosnode list | grep '/record_')


