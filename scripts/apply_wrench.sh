#!/bin/bash

if [[ $# -ne 6 ]]; then
    echo "Illegal number of parameters!"
    exit 2
fi

echo "Applying wrench:"
echo "Force xyz: ($1 $2 $3) Torque xyz: ($4 $5 $6) "

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/set_cartesian_wrench geometry_msgs/WrenchStamped "
wrench:
  force:
    x: $1
    y: $2
    z: $3
  torque:
    x: $4
    y: $5
    z: $6"

echo "Wrench applied."
