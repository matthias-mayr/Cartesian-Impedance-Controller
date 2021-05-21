#!/bin/bash

if [[ $# -ne 7 ]]; then
    echo "Illegal number of parameters!"
    exit 2
fi

echo "Setting stiffness:"
echo "K_t= ($1 $2 $3), K_r= ($4 $5 $6), K_n= $7 "

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/cartesian_impedance_parameters cartesian_impedance_controller/CartesianImpedanceControlMode "cartesian_stiffness: {x: $1, y: $2, z: $3, a: $4, b: $5, c: $6}
nullspace_stiffness: $7
q_d_nullspace: {q1: 1.2701715984878614, q2: 0.06980988466741955, q3: 0.2716601287156033, q4: -0.9147682346772754, q5: 0.3409470977823119, q6: 1.175819569186606, q7: 0.69816013366456}" > /dev/null 2>&1

echo "Stiffness set."



