#!/bin/bash

#Sliding motion

./change_stiffness.sh 100 100 50 50 50 50 0
python python_scripts/wait_for_stiffness.py 100 100 50 50 50 50 0
