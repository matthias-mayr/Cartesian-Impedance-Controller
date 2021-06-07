#!/bin/bash

#Singularity stuff

./singularity_config1.sh

./change_stiffness.sh 200 200 200 100 100 100 0

python python_scripts/wait_for_stiffness.py 200 200 200 100 100 100 0


./change_goal.sh -0.495254188776 0.427890121937 1.39696657658 0.50154197216 -0.416139572859 -0.265110373497 -0.710633516312

python python_scripts/wait_for_not_moving.py
