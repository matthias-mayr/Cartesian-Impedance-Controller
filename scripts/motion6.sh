#!/bin/bash



#Check if end effector is reached within allocated time
#----------------------------------------------------------
#time_start=$SECONDS
#while : ; do 
#out=$(./check_pose.py)
#if (( $out == 1 || $(($SECONDS - $time_start)) > 4)); then
#break 2
#else
#echo $((10-$SECONDS+$time_start))
#fi
#done #end while
#------------------------------------------------------------

#increase cartesian stiffness in steps

#step 1

#./change_damping.sh 0.85 0.85 0.85 0.85 0.85 0.85
./change_damping.sh 0.7 0.7 0.7 0.7 0.7 0.7

./change_stiffness.sh 200 200 200 100 100 100 0
sleep 4
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
sleep 7
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0
sleep 4 


#step 2

./change_stiffness.sh 400 400 400 100 100 100 0
sleep 4
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
sleep 7
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0
sleep 4


#step 3

./change_stiffness.sh 600 600 600 100 100 100 0
sleep 4
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
sleep 7
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0
sleep 4


#step 4

./change_stiffness.sh 800 800 800 100 100 100 0
sleep 4
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
sleep 7
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0
sleep 4


sleep 5





