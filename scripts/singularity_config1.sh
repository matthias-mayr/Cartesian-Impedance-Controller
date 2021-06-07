#!/bin/bash


a1=50
a2=50
a3=50
a4=30
a5=30
a6=30
a7=20


echo "Setting stiffness:"
echo "K_t= ($a1 $a2 $a3), K_r= ($a4 $a5 $a6), K_n= $a7 "

./change_goal.sh  -0.537901341915 -0.0461745485663 1.59478247166 -0.449824333191 -0.00490805599838 0.73346978426 0.509564578533 
rostopic pub --once /bh/CartesianImpedance_trajectory_controller/cartesian_impedance_parameters cartesian_impedance_controller/CartesianImpedanceControlMode "cartesian_stiffness: {x: $a1, y: $a2, z: $a3, a: $a4, b: $a5, c: $a6}
nullspace_stiffness: $a7
q_d_nullspace: {q1: -0.263318300247, q2: -0.0664568990469, q3: 0.0321720726788, q4: -1.39160084724, q5: -1.81047570705, q6: 0.017166743055, q7: 2.29075717926}" > /dev/null 2>&1



echo "Stiffness set."

python python_scripts/wait_for_stiffness.py $a1 $a2 $a3 $a4 $a5 $a6 $a7
python python_scripts/wait_for_not_moving.py
