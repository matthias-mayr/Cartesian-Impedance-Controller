#!/bin/bash

trap 'exit 130' INT

echo "This experimental motion consists of some trajectories with intermidiate pauses."


while : ; do 
nr_files=$(ls --ignore="*record.sh" -1 | grep "motion" | wc -l)
echo "Pick a NUMBER(1-$nr_files). Available:"
ls --ignore="*record.sh" -1 | grep "motion"

read nr 


if (( $nr >= 1 && $nr <= $nr_files )); then

echo "You picked motion$nr.sh " && break 2

else

echo "Incorrect input. Try again" 

fi 


done #end while
 
read -r -p "Do you want to save the experiment in a bag-file? [y/N] " response
case "$response" in
    [yY][eE][sS]|[yY]) 
echo "YES"
read -r -p "is the experiment performed in simulation or robot? [s/r] " in_
case "$in_" in
    [sS][iI][mM]|[sS]) 
	echo "You picked SIMULATION"
echo "Press any key to start"
read step1
	./home_position.sh
      ./log_data.sh 100 motion${nr}_simulation & ./motion${nr}.sh
        ;;
    *)

	echo "You picked ROBOT"
echo "Press any key to start"
read step1
echo "Starting motion ..."
	./home_position.sh
      	./log_data.sh 100 motion${nr}_robot &./motion${nr}.sh
        ;;
esac
        ;;
    *)
	echo "NO"

echo "Press any key to start"
read step1
echo "Starting motion"
	./home_position.sh
       ./motion${nr}.sh
        ;;
esac






