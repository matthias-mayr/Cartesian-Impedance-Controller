#!/bin/bash

# PEG IN HOLE 
#-----------

## Get into position
./change_stiffness.sh 1000 1000 1000 50 50 50 0
python python_scripts/wait_for_stiffness.py 1000 1000 1000 50 50 50 0
./change_goal.sh -0.6 0 1 1 0 0 0
python python_scripts/wait_for_not_moving.py
./change_goal.sh -0.6 -0.02 0.82 1 0 0 0

./change_stiffness.sh 100 100 5 50 50 50 0
python python_scripts/wait_for_stiffness.py 200 200 5 10 10 10 0


rosservice call /cartesian_trajectory_generator/overlay_motion "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
motion: 1
radius: 0.05
path_distance: 0.01
path_velocity: 0.02
allow_decrease: false
dir: {x: 0.0, y: 0.0, z: 0.0}" 


./apply_wrench.sh 0 0 2 0 0 0
./change_goal.sh -0.6 -0.0 0.82 1 0 0 0
height_limit=0.77
while :
do
is_above=$(python python_scripts/is_above_height.py $height_limit)

if [ "$is_above" -eq "0" ]; then #if we get in the hole
echo "Peg is inserted"
./apply_wrench.sh 0 0 0 0 0 0
rosservice call /cartesian_trajectory_generator/overlay_motion "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
motion: 0
radius: 0.1
path_distance: 0.1
path_velocity: 0.02
allow_decrease: false
dir: {x: 0.0, y: 0.0, z: 0.0}"

./change_stiffness.sh 50 50 200 50 50 50 0
./change_goal.sh -0.6 0.0 1.0 1 0 0 0

exit; 
fi #end if


done #while

#stop searching if z=0.74
