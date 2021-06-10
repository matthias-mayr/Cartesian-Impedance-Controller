#!/bin/bash

./change_stiffness.sh 150 150 150 50 50 50 0

python python_scripts/wait_for_stiffness.py 150 150 150 50 50 50 0

./change_goal.sh -0.5 0 1.0 1 0 0 0

python python_scripts/wait_for_not_moving.py 

sleep 5

./change_goal.sh -0.5 -0.5 1.0 1 0 0 0

python python_scripts/wait_for_not_moving.py 

sleep 5

./change_stiffness.sh 100 100 100 50 50 50 0

python python_scripts/wait_for_stiffness.py 100 100 100 50 50 50 0

./change_goal.sh -0.5 0 1.0 1 0 0 0

python python_scripts/wait_for_not_moving.py 

sleep 5

python python_scripts/wait_for_not_moving.py 

./change_goal.sh -0.5 -0.5 1.0 1 0 0 0

python python_scripts/wait_for_not_moving.py 

sleep 5
