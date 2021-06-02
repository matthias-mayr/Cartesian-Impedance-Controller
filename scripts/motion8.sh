#!/bin/bash




#Apply FORCES
#------------

waiting_after_wrench=40
waiting_after_release=8
waiting_after_stiffness=25

force_applied=-5
# Stiffness configuration #1
sleep 15
./change_stiffness.sh 200 200 50 100 100 100 0

sleep $waiting_after_stiffness

./apply_wrench.sh 0 0 $force_applied 0 0 0

sleep $waiting_after_wrench

./apply_wrench.sh 0 0 0 0 0 0

sleep $waiting_after_release



# Stiffness configuration #2
./change_stiffness.sh 200 200 100 100 100 100 0

sleep $waiting_after_stiffness

./apply_wrench.sh 0 0 $force_applied 0 0 0

sleep $waiting_after_wrench

./apply_wrench.sh 0 0 0 0 0 0

sleep $waiting_after_release


# Stiffness configuration #2
./change_stiffness.sh 200 200 200 100 100 100 0

sleep $waiting_after_stiffness

./apply_wrench.sh 0 0 $force_applied 0 0 0

sleep $waiting_after_wrench

./apply_wrench.sh 0 0 0 0 0 0

sleep $waiting_after_release
