#!/bin/bash

rostopic pub /bh/PositionJointInterface_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ["bh_joint_1", "bh_joint_2", "bh_joint_3", "bh_joint_4", "bh_joint_5", "bh_joint_6", "bh_joint_7"]
points:
- positions: [0, -0.2037, -0.1289, -1.6373, 0, 1.471, 0]
  velocities: [0, 0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 1.0, nsecs: 0}" --once &

rostopic pub /hh/PositionJointInterface_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ["hh_joint_1", "hh_joint_2", "hh_joint_3", "hh_joint_4", "hh_joint_5", "hh_joint_6", "hh_joint_7"]
points:
- positions: [0, -0.2037, -0.1289, -1.6373, 0, 1.471, 0]
  velocities: [0, 0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 1.0, nsecs: 0}" --once