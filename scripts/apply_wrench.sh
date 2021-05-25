#!/bin/bash

if [[ $# -ne 6 ]]; then
    echo "Illegal number of parameters!"
    exit 2
fi

echo "Applying wrench:"
echo "Force xyz: ($1 $2 $3) Torque xyz: ($4 $5 $6) "

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/set_cartesian_wrench cartesian_impedance_controller/CartesianWrench "{f_x: $1, f_y: $2, f_z: $3, tau_x: $4, tau_y: $5, tau_z: $6}"
