#!/bin/bash


if [[ $# -ne 7 ]]; then
    echo "Illegal number of parameters!"
    exit 2
fi

echo "Setting new goal:"
echo "position: ($1 $2 $3) orientation: ($4 $5 $6 $7) "
rostopic pub --once /bh/CartesianImpedance_trajectory_controller/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position: 
    x: $1
    y: $2
    z: $3
  orientation: 
    x: $4
    y: $5
    z: $6
    w: $7" > /dev/null 2>&1 

echo "Goal set."
