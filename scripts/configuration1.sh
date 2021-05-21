#!/bin/bash


echo "Press any key to run";
read start;
echo "Changing reference pose..."

rostopic pub --once /bh/CartesianImpedance_trajectory_controller/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position: 
    x: -0.5
    y: -0.5
    z: 0.9
  orientation: 
    x: 0.464521359639
    y: 0.217117400384
    z: 0.118611776418
    w: 0.850300645292"


echo "Done."




