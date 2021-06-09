#!/bin/bash

if [[ $# -ne 6 ]]; then
    echo "Illegal number of parameters!"
    exit 2
fi

echo "Setting damping factors:"
echo "D_t= ($1 $2 $3), D_r= ($4 $5 $6)"

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/set_damping_factors cartesian_impedance_controller/CartesianImpedanceControlMode "cartesian_damping: {x: $1, y: $2, z: $3, a: $4, b: $5, c: $6}" > /dev/null 2>&1

echo "Damping set."



