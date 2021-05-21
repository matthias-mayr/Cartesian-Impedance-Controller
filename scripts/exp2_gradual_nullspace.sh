#!/bin/bash


echo "Press any key to run"
read step1

./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0 & ./change_stiffness 150 150 150 30 30 30 25

echo "Increase nullspace stiffness?" 
read step1
./change_stiffness 150 150 150 30 30 30 50

echo "Increase nullspace stiffness?" 
read step1
./change_stiffness 150 150 150 30 30 30 100

echo "Increase nullspace stiffness?" 
read step1
./change_stiffness 150 150 150 30 30 30 200
