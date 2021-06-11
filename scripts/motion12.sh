#!/bin/bash

## NULLSPACE COMPLIANCE

./change_stiffness.sh 300 300 300 200 200 200 0
sleep 5

./change_stiffness.sh 2000 2000 2000 200 200 200 0


python python_scripts/wait_for_stiffness.py 2000 2000 2000 200 200 200 0

echo "Sleeping for 60 seconds"
sleep 60
