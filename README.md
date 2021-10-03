# Cartesian Impedance Controller

## Description
This project is an implementation of aCartesian impedance controller. It is a type of control strategy that sets a dynamic relationship between contact forces and the position of a robot arm, making it suitable for collaborative robots. 

The controller has been developed using the seven degree-of-freedom (DoF) robot arm called LBR iiwa by KUKA AG. It is, however, universal and should therefore work for other seven DoF robot arms, such as the Panda by Franka Emika.

## Features

- Configurable stiffness values along all Cartesian dimensions
- Configurable damping factors along all Cartesian dimensions
- Change desired pose in run-time
- Apply Cartesian forces in run-time

![](./res/flowchart.png)

Before new reference stiffness, damping and/or Cartesian forces are sent to the controller, they are saturated to reasonable limits and filtered through a low-pass filter. This is done in order to make the transition between reference values safer and less jerky. The control signals goes through a rate-limiter for similar reasons, before being sent to the robot actuators.

## Limitations

- Friction not accounted for
- Stiffness and damping values along the Cartesian dimensions are uncoupled
- Only tested for the LBR iiwa model

## Prerequisites

- [ROS melodic](https://www.ros.org/)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)
- bh_robot (provide link)
- kuka-fri (provide link)
- corrade (provide link)
- mc_rbdyn_urdf (provide link)
- SpaceVecAlg (provide link)
- robot_controllers (provide link)
- cartesian_trajectory_generator (provide link)

## Usage

Open two terminals:

1. $ `roscore`
2. $ `mon launch bh_robot bringup.launch run_moveit:=true`


### Changing parameters in run-time (with ROS-topics)

The controller uses some default values which can be configured during run-time. Some parameters that can be configured are stiffness values, damping factors, reference pose and Cartesian wrenches in the end-effector.

#### Stiffness and null-space

The stiffness and null-space can be configured by using the following command:

$ `rostopic pub --once /bh/CartesianImpedance_trajectory_controller/set_stiffness cartesian_impedance_controller/CartesianImpedanceControlMode "cartesian_stiffness: {x: $1, y: $2, z: $3, a: $4, b: $5, c: $6}
nullspace_stiffness: $7
q_d_nullspace: {q1: 1.270, q2: 0.069, q3: 0.271, q4: -0.915, q5: 0.341, q6: 1.176, q7: 0.698}"`

where the arguments 

`$1` - translational stiffness along x-dimension  
`$2` - translational stiffness along y-dimension  
`$3` - translational stiffness along z-dimension  
`$4` - rotational stiffness along x-dimension  
`$5` - rotational stiffness along y-dimension  
`$6` - rotational stiffness along z-dimension  
`$7` - null-space stiffness  

can be chosen to reasonable values. the desired null-space configuration `q_d_nullspace` is by default chosen to some configuration defined as "home" position, but can also be made configurable as the other parameters.

#### Damping factors

The damping factors can be configured using the following command: 

$ `rostopic pub --once /bh/CartesianImpedance_trajectory_controller/set_damping_factors cartesian_impedance_controller/CartesianImpedanceControlMode "cartesian_damping: {x: $1, y: $2, z: $3, a: $4, b: $5, c: $6}
nullspace_damping: $7"`

where the arguments 

`$1` - translational damping factor along x-dimension  
`$2` - translational damping factor along y-dimension  
`$3` - translational damping factor along z-dimension  
`$4` - rotational damping factor along x-dimension  
`$5` - rotational damping factor along y-dimension  
`$6` - rotational damping factor along z-dimension  
`$7` - null-space damping factor  

can be chosen to reasonable values.

#### End-effector pose

A new reference pose can be sent using the following command:

$ `rostopic pub --once /bh/CartesianImpedance_trajectory_controller/goal geometry_msgs/PoseStamped "header:
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
    w: $7"`

where the arguments `$1`, `$2` and `$3` are the positional coordinates and `$4`, `$5`, `$6` and `$7` describe the orientation with unit-quaternions.


#### Cartesian wrenches

A Cartesian wrench can be applied using the following command:

`rostopic pub --once /bh/CartesianImpedance_trajectory_controller/set_cartesian_wrench geometry_msgs/WrenchStamped "
wrench:
  force:
    x: $1
    y: $2
    z: $3
  torque:
    x: $4
    y: $5
    z: $6"`

where the arguments

`$1` - applied force along the x-axis  
`$2` - applied force along the y-axis  
`$3` - applied force along the z-axis  
`$4` - applied torque along the x-axis  
`$5` - applied torque along the y-axis  
`$6` - applied torque along the z-axis  

can be chosen to reasonable values.


### Changing parameters in run-time (with dynamic_reconfigure)

Open a new terminal and run:

`rosrun rqt_reconfigure rqt_reconfigure`

In the GUI to the left there are multiple tabs to choose from.

#### Stiffness

The stiffness can be configured under the tab 'stiffness_reconfigure'. To update the newly selected stiffness, press the button 'apply_stiffness'.

#### Damping factors

The damping factors can be configured under the tab 'damping_factors_reconfigure'. To update the newly selected damping factors, press the button 'apply_damping_factors'.

#### End-effector pose

The desired end-effector pose can be selected under the tab 'cartesian_trajectory_generator/cartesian_trajectory_generator'. To update the newly selected pose, press the button 'ready_to_send'.

#### Cartesiann wrenches

A Cartesian wrench can be applied under the tab 'cartesian_wrench_reconfigure'. To apply the newly selected wrench press the button 'apply_wrench'.