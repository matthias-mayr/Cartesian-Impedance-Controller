#!/bin/bash


#Singularity stuff

./change_stiffness.sh 1 1 1 1 1 1 0

python python_scripts/wait_for_stiffness.py 1 1 1 1 1 1 0

echo "You can start pertubate the robot now"

sleep 60
