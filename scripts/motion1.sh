#!/bin/bash

#Sliding motion

./home_position_sliding.sh
read a
echo "Press ready to start"
./apply_wrench.sh 0 0 5 0 0 0
python python_scripts/wait_for_wrench.py 0 0 5 0 0 0



#Table : z: 0.707
