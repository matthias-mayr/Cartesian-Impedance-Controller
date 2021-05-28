#!/bin/bash


#Apply TORQUES
#------------

# Stiffness configuration #1
./change_stiffness.sh 200 200 200 5 50 50 0

sleep 2

./apply_wrench.sh 0 0 0 7.5 0 0

sleep 5

./apply_wrench.sh 0 0 0 0 0 0

sleep 5

# Stiffness configuration #2
./change_stiffness.sh 200 200 200 10 50 50 0

sleep 2

./apply_wrench.sh 0 0 0 7.5 0 0

sleep 5

./apply_wrench.sh 0 0 0 0 0 0


# Stiffness configuration #3
./change_stiffness.sh 200 200 200 20 50 50 0
sleep 2

./apply_wrench.sh 0 0 0 7.5 0 0

sleep 5

./apply_wrench.sh 0 0 0 0 0 0



# Stiffness configuration #4
./change_stiffness.sh 200 200 200 50 50 50 0
sleep 2

./apply_wrench.sh 0 0 7.5 0 0 0

sleep 5

./apply_wrench.sh 0 0 0 0 0 0



# Stiffness configuration #5
./change_stiffness.sh 200 200 200 100 50 50 0
sleep 2

./apply_wrench.sh 0 0 0 7.5 0 0

sleep 5

./apply_wrench.sh 0 0 0 0 0 0



# Stiffness configuration #6
./change_stiffness.sh 200 200 200 100 50 50 0
sleep 2

./apply_wrench.sh 0 0 0 7.5 0 0

sleep 5

./apply_wrench.sh 0 0 0 0 0 0


