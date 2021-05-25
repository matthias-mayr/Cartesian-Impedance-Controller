#!/bin/bash


./change_stiffness.sh 200 200 200 100 100 100 0
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
sleep 3
#Check if end effector is reached within allocated time
#----------------------------------------------------------
time_start=$SECONDS
while : ; do 
out=$(./check_pose.py)
if (( $out == 1 || $(($SECONDS - $time_start)) > 20)); then
break 2
else
echo $((30-$SECONDS+$time_start))
fi
done #end while
#------------------------------------------------------------

#increase rotational stiffness
./change_stiffness.sh 200 200 200 200 200 200 0
sleep 3
./change_stiffness.sh 200 200 200 300 300 300 0
sleep 3
./change_stiffness.sh 200 200 200 400 400 400 0
#reset rot. stiffness & increase translational stiffness 
sleep 3
./change_stiffness.sh 300 300 300 100 100 100 0
sleep 3
./change_stiffness.sh 400 400 400 100 100 100 0
sleep 3
./change_stiffness.sh 400 400 400 100 100 100 0
sleep 3
./change_stiffness.sh 500 500 500 100 100 100 0

echo "Continue?"
read r1
./change_stiffness.sh 550 550 550 100 100 100 0
echo "Continue?"
read r1
./change_stiffness.sh 600 600 600 100 100 100 0

echo "Continue?"
read r1
./change_stiffness.sh 650 650 650 100 100 100 0

echo "Continue?"
read r1
./change_stiffness.sh 700 700 700 100 100 100 0

echo "Motion complete"

