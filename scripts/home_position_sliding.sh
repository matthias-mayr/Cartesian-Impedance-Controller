#!/bin/bash

echo "Returning to the \"home\" position";

echo "Setting stiffness:"

a1=600
a2=600
a3=600
a4=100
a5=100
a6=100
a7=20
echo "K_t= ($a1 $a2 $a3), K_r= ($a4 $a5 $a6), K_n= $a7 "

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/cartesian_impedance_parameters cartesian_impedance_controller/CartesianImpedanceControlMode "cartesian_stiffness: {x: $a1, y: $a2, z: $a3, a: $a4, b: $a5, c: $a6}
nullspace_stiffness: $a7
q_d_nullspace: {q1: 1.598 , q2: 0.329, q3: -0.15, q4: -1.004 , q5: 0.582, q6: 0.887, q7: 0.449}" > /dev/null 2>&1 &./change_goal.sh -0.533 -0.421 0.804 1 0 0 0

echo "Stiffness set."

python python_scripts/wait_for_stiffness.py $a1 $a2 $a3 $a4 $a5 $a6 $a7
python python_scripts/wait_for_not_moving.py
echo "Setting stiffness:"

b1=600
b2=600
b3=1500
b4=100
b5=100
b6=100
b7=0
echo "K_t= ($b1 $b2 $b3), K_r= ($b4 $b5 $b6), K_n= $b7 "

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/cartesian_impedance_parameters cartesian_impedance_controller/CartesianImpedanceControlMode "cartesian_stiffness: {x: $b1, y: $b2, z: $b3, a: $b4, b: $b5, c: $b6}
nullspace_stiffness: $b7
q_d_nullspace: {q1: 1.598 , q2: 0.329, q3: -0.15, q4: -1.004 , q5: 0.582, q6: 0.887, q7: 0.449}" > /dev/null 2>&1

echo "Stiffness set."

python python_scripts/wait_for_stiffness.py $b1 $b2 $b3 $b4 $b5 $b6 $b7

#Get closer and closer to the table
./change_goal.sh -0.533 -0.421 0.72 1 0 0 0
sleep 2
python python_scripts/wait_for_not_moving.py
./change_goal.sh -0.533 -0.421 0.71 1 0 0 0
sleep 2
python python_scripts/wait_for_not_moving.py

