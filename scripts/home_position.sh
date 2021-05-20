#!/bin/bash

echo "Returning to the \"home\" position";


./change_goal.sh -0.5 -0.5 0.8 1 0 0 0 & ./change_stiffness.sh 150 150 150 50 50 50 20


echo "Press any key ONLY IF the robot is returned.";

read step2;

./change_stiffness.sh 200 200 200 100 100 100 0



