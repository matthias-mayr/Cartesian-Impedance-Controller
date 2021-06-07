#!/bin/bash

#testing singularity SIMULATION


x=-1
y=-0.2
z=0.9

wait_time=5

./change_stiffness.sh 80 80 80 30 30 30 0
python python_scripts/wait_for_stiffness.py 80 80 80 30 30 30 0

./change_goal.sh $x $y $z 1 0 0 0
python python_scripts/wait_for_not_moving.py
#Add some delay
python python_scripts/wait_sim_time.py $wait_time

./home_position.sh

./change_stiffness.sh 200 200 200 50 50 50 0
python python_scripts/wait_for_stiffness.py 200 200 200 100 100 100 0

./change_goal.sh $x $y $z 1 0 0 0
python python_scripts/wait_for_not_moving.py
#Add some delay
python python_scripts/wait_sim_time.py $wait_time

./home_position.sh



./change_stiffness.sh 500 500 500 100 100 100 0
python python_scripts/wait_for_stiffness.py 500 500 500 100 100 100 0

./change_goal.sh $x $y $z 1 0 0 0
python python_scripts/wait_for_not_moving.py

python python_scripts/wait_sim_time.py $wait_time
