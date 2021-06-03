#!/bin/bash




#Apply FORCES
#------------


waiting_after_wrench=40
waiting_after_release=8

force_applied=-5
# Stiffness configuration #1
./change_stiffness.sh 200 200 50 100 100 100 0
python python_scripts/wait_for_stiffness.py 200 200 50 100 100 100 0.05


./apply_wrench.sh 0 0 $force_applied 0 0 0

python python_scripts/wait_for_wrench.py 0 0 $force_applied 0 0 0
python python_scripts/wait_for_not_moving.py

./apply_wrench.sh 0 0 0 0 0 0

python python_scripts/wait_for_wrench.py 0 0 0 0 0 0
python python_scripts/wait_for_not_moving.py



# Stiffness configuration #2
./change_stiffness.sh 200 200 100 100 100 100 0

python python_scripts/wait_for_stiffness.py 200 200 100 100 100 100 0.05


./apply_wrench.sh 0 0 $force_applied 0 0 0

python python_scripts/wait_for_wrench.py 0 0 $force_applied 0 0 0
python python_scripts/wait_for_not_moving.py

./apply_wrench.sh 0 0 0 0 0 0

python python_scripts/wait_for_wrench.py 0 0 0 0 0 0
python python_scripts/wait_for_not_moving.py



# Stiffness configuration #2
./change_stiffness.sh 200 200 200 100 100 100 0

python python_scripts/wait_for_stiffness.py 200 200 200 100 100 100 0.05


./apply_wrench.sh 0 0 $force_applied 0 0 0

python python_scripts/wait_for_wrench.py 0 0 $force_applied 0 0 0
python python_scripts/wait_for_not_moving.py

./apply_wrench.sh 0 0 0 0 0 0

python python_scripts/wait_for_wrench.py 0 0 0 0 0 0
python python_scripts/wait_for_not_moving.py

