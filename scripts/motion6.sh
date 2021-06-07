#!/bin/bash

#damping
d=0.7
./change_damping.sh $d $d $d $d $d $d


#increase cartesian stiffness in steps

#step 1
sleep_time_goal=20
sleep_stiffness=10

sleep 5
./change_stiffness.sh 250 250 250 100 100 100 0
sleep $sleep_stiffness
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
sleep $sleep_time_goal
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0 & ./change_stiffness.sh 250 250 250 100 100 100 5
sleep 1
./change_stiffness.sh 250 250 250 100 100 100 0
sleep $sleep_time_goal 



#step 3

./change_stiffness.sh 600 600 600 100 100 100 0
sleep $sleep_stiffness
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
sleep $sleep_time_goal
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0 &./change_stiffness.sh 600 600 600 100 100 100 5
sleep 1
./change_stiffness.sh 600 600 600 100 100 100 0
sleep $sleep_time_goal


./change_stiffness.sh 1000 1000 1000 100 100 100 0
sleep $sleep_stiffness
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0 
sleep $sleep_time_goal
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0 &./change_stiffness.sh 1000 1000 1000 100 100 100 5
sleep 1
./change_stiffness.sh 1000 1000 1000 100 100 100 0
sleep $sleep_time_goal


sleep 5





