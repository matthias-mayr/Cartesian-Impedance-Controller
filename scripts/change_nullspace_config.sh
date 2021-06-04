#!/bin/bash

if [[ $# -ne 7 ]]; then
    echo "Illegal number of parameters!"
    exit 2
fi

echo "Setting nullspace configuration..."

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/nullspace_configuration cartesian_impedance_controller/CartesianImpedanceControlMode "q_d_nullspace: {q1: $1, q2: $2, q3: $3, q4: $4, q5: $5, q6: $6, q7: $7}" > /dev/null 2>&1

echo "Configuration set."

