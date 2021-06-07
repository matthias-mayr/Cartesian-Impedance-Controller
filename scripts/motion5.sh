#!/bin/bash

#!/bin/bash


#DONE-------------------

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

#damping
d=1
./change_damping.sh $d $d $d $d $d $d


#increase cartesian stiffness in steps

#step 1
sleep_time_goal=20
sleep_stiffness=14

python python_scripts/wait_sim_time.py 7
./change_stiffness.sh 250 250 250 100 100 100 0
python python_scripts/wait_sim_time.py $sleep_stiffness

#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
python python_scripts/wait_sim_time.py $sleep_time_goal
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0 & ./change_stiffness.sh 250 250 250 100 100 100 5
python python_scripts/wait_sim_time.py 1
./change_stiffness.sh 250 250 250 100 100 100 0
python python_scripts/wait_sim_time.py $sleep_time_goal



#step 3

./change_stiffness.sh 600 600 600 100 100 100 0
python python_scripts/wait_sim_time.py $sleep_stiffness
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0
python python_scripts/wait_sim_time.py $sleep_time_goal
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0 &./change_stiffness.sh 600 600 600 100 100 100 5
python python_scripts/wait_sim_time.py 1
./change_stiffness.sh 600 600 600 100 100 100 0
python python_scripts/wait_sim_time.py $sleep_time_goal


./change_stiffness.sh 1000 1000 1000 100 100 100 0
python python_scripts/wait_sim_time.py $sleep_stiffness
#go forward
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0 
python python_scripts/wait_sim_time.py $sleep_time_goal
#go back
./change_goal.sh -0.5 -0.5 1 1.0 0.0 0.0 0.0 &./change_stiffness.sh 1000 1000 1000 100 100 100 5
python python_scripts/wait_sim_time.py 1
./change_stiffness.sh 1000 1000 1000 100 100 100 0
python python_scripts/wait_sim_time.py $sleep_time_goal






