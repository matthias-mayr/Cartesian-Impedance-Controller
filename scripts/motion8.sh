#!/bin/bash


#!/bin/bash

./change_stiffness.sh 200 200 200 100 100 100 0
python python_scripts/wait_for_stiffness.py 200 200 200 100 100 100 0

./change_goal.sh -0.7 0.0 1 0.707 0.0 0.0 -0.707 
sleep 5
python python_scripts/wait_for_not_moving.py







#Apply FORCES
#------------
force_applied=10

# Stiffness configuration #1
stf1=200
./change_stiffness.sh 200 200 $stf1 100 100 100 0
python python_scripts/wait_for_stiffness.py 200 200 $stf1 100 100 100 0


./apply_wrench.sh 0 0 $force_applied 0 0 0

python python_scripts/wait_for_wrench.py 0 0 $force_applied 0 0 0
python python_scripts/wait_for_not_moving.py

./apply_wrench.sh 0 0 0 0 0 0 & ./change_stiffness.sh 200 200 $stf1 100 100 100 20
python python_scripts/wait_for_stiffness.py 200 200 $stf1 100 100 100 20
python python_scripts/wait_for_wrench.py 0 0 0 0 0 0
python python_scripts/wait_for_not_moving.py



