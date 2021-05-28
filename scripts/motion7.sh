#!/bin/bash

#Apply FORCES
#------------
force=5
sleep_time_after_apply=30
# Stiffness configuration #1
./change_stiffness.sh 100 100 50 50 50 50 5

sleep 2

./apply_wrench.sh 0 0 $force 0 0 0

sleep $sleep_time_after_apply

./apply_wrench.sh 0 0 0 0 0 0

sleep 5



# Stiffness configuration #2
./change_stiffness.sh 100 100 200 50 50 50 5
sleep 2

./apply_wrench.sh 0 0 $force 0 0 0

sleep $sleep_time_after_apply

./apply_wrench.sh 0 0 0 0 0 0




# Stiffness configuration #3
./change_stiffness.sh 100 100 800 50 50 50 5
sleep 2

./apply_wrench.sh 0 0 $force 0 0 0

sleep $sleep_time_after_apply

./apply_wrench.sh 0 0 0 0 0 0

