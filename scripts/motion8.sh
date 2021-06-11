#!/bin/bash

# PEG IN HOLE 
#-----------

##Peg surface z: 0.807

## Get into position
./change_stiffness.sh 2000 2000 2000 50 50 5 0
python python_scripts/wait_for_stiffness.py 1500 1500 1500 200 200 5 0
./change_goal.sh -0.6 0 1 1 0 0 0
python python_scripts/wait_for_not_moving.py
./change_goal.sh -0.63 -0.03 0.815 1 0 0 0

./change_stiffness.sh 350 350 0.1 100 100 5 0
python python_scripts/wait_for_stiffness.py 350 350 0.1 100 100 5 0


rosservice call /cartesian_trajectory_generator/overlay_motion "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
motion: 1
radius: 0.03
path_distance: 0.075
path_velocity: 0.005
allow_decrease: true
dir: {x: 0.0, y: 0.0, z: 0.0}" 


./apply_wrench.sh 0 0 5 0 0 0
./change_goal.sh -0.6 -0.0 0.85 1 0 0 0
height_limit=0.788

while :
do
is_above=$(python python_scripts/is_above_height.py $height_limit)

if [ "$is_above" -eq "0" ]; then #if we get in the hole
echo "Peg is inserted"
./change_stiffness.sh 1 1 0.1 10 10 10 0 & (rosservice call /cartesian_trajectory_generator/overlay_motion "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
motion: 0
radius: 0.1
path_distance: 0.1
path_velocity: 0.02
allow_decrease: false
dir: {x: 0.0, y: 0.0, z: 0.0}"
) &./apply_wrench.sh 0 0 0 0 0 0

pos_string=$(python python_scripts/get_position.py)
IFS=' ' read -r -a pos <<< "$pos_string"
x=${pos[0]}
y=${pos[1]}
z=${pos[2]}
./change_goal.sh $x $y $z 1 0 0 0 
./change_stiffness.sh 1 1 600 100 100 100 0
python python_scripts/wait_for_stiffness.py 1 1 500 100 100 100 0

./change_goal.sh $x $y 0.9 1 0 0 0 

python python_scripts/wait_for_not_moving.py
./change_stiffness.sh 2000 2000 2000 50 50 5 0

exit; 
fi #end if
done #while

#stop searching if z=0.74
