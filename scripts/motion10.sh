#!/bin/bash

## ROTATIONAL COMPLIANCE

./change_stiffness.sh 2000 2000 2000 0 0 0 0


python python_scripts/wait_for_stiffness.py 2000 2000 2000 0 0 0 0

echo "Sleeping for 60 seconds"
sleep 60
