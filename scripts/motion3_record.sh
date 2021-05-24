#!/bin/bash

trap 'exit 130' INT

echo "This experimental motion consists of some trajectories with intermidiate pauses."

./home_position.sh

read -r -p "Do you want to save the experiment in a bag-file? [y/N] " response
case "$response" in
    [yY][eE][sS]|[yY]) 
echo "YES"
read -r -p "is the experiment performed in simulation or robot? [s/r] " in_
case "$in_" in
    [sS][iI][mM]|[sS]) 
	echo "SIMULATION"
echo "Press any key to start"
read step1

      ./log_data.sh 100 motion3_simulation & ./motion3.sh
        ;;
    *)

	echo "ROBOT"
echo "Press any key to start"
read step1
      	./log_data.sh 100 motion3_robot &./motion3.sh
        ;;
esac
        ;;
    *)
	echo "NO"

echo "Press any key to start"
read step1

       ./motion3.sh
        ;;
esac






