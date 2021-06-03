#!/bin/bash

#Apply FORCES
#------------
force_applied=-10

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



# Stiffness configuration #2
stf2=100
./change_stiffness.sh 200 200 $stf2 100 100 100 0

python python_scripts/wait_for_stiffness.py 200 200 $stf2 100 100 100 0


./apply_wrench.sh 0 0 $force_applied 0 0 0

python python_scripts/wait_for_wrench.py 0 0 $force_applied 0 0 0
python python_scripts/wait_for_not_moving.py

./apply_wrench.sh 0 0 0 0 0 0 & ./change_stiffness.sh 200 200 $stf2 100 100 100 20

python python_scripts/wait_for_stiffness.py 200 200 $stf2 100 100 100 20
python python_scripts/wait_for_wrench.py 0 0 0 0 0 0
python python_scripts/wait_for_not_moving.py



# Stiffness configuration #3
stf3=50
./change_stiffness.sh 200 200 $stf3 100 100 100 0

python python_scripts/wait_for_stiffness.py 200 200 $stf3 100 100 100 0

./apply_wrench.sh 0 0 $force_applied 0 0 0

python python_scripts/wait_for_wrench.py 0 0 $force_applied 0 0 0
python python_scripts/wait_for_not_moving.py

./apply_wrench.sh 0 0 0 0 0 0 &./change_stiffness.sh 200 200 $stf3 100 100 100 20

python python_scripts/wait_for_stiffness.py 200 200 $stf3 100 100 100 20
python python_scripts/wait_for_wrench.py 0 0 0 0 0 0
python python_scripts/wait_for_not_moving.py

