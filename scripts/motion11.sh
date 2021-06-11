#!/bin/bash

## TRANSLATIONAL COMPLIANCE

./change_stiffness.sh 3 3 3 200 200 200 0


python python_scripts/wait_for_stiffness.py 3 3 3 200 200 200 0

echo "Sleeping for 60 seconds"
sleep 60
