# Cartesian Impedance Controller

## Description
This project is an implementation of aCartesian impedance controller. It is a type of control strategy that sets a dynamic relationship between contact forces and the position of a robot arm, making it suitable for collaborative robots. 

The controller has been developed using the seven degree-of-freedom (DoF) robot arm called LBR iiwa by KUKA AG. It is, however, universal and should therefore work for other seven DoF robot arms, such as the Panda by Franka Emika.

## Prerequisites
### Required
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

### ROS Controller
We use `RBDyn` to calculate forward kinematics and the Jacobian.

- [RBDyn](https://github.com/jrl-umi3218/RBDyn)
- mc_rbdyn_urdf (provide link)
- SpaceVecAlg (provide link)
- [ROS](https://www.ros.org/)

The below-mentioned installation steps are automated in `scripts/install_dependencies.sh`:

**SpaceVecAlg**
```bash
cd /source/directory
git clone --recursive https://github.com/costashatz/SpaceVecAlg.git
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

**RBDyn**
```
cd /source/directory
git clone --recursive https://github.com/costashatz/RBDyn.git
cd RBDyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

**mc_rbdyn_urdf**
```bash
cd /source/directory
git clone --recursive https://github.com/costashatz/mc_rbdyn_urdf.git
cd mc_rbdyn_urdf
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
```

## Controller Usage in ROS
For using this controller within ROS, a catkin workspace is needed:

```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
cd src
git clone git@git.cs.lth.se:robotlab/cartesian-impedance-controller.git
```

### Configuration file
```YAML
CartesianImpedance_trajectory_controller:
  type: cartesian_impedance_controller/CartesianImpedanceController
  joints:                               # Joints to control
    - iiwa_joint_1
    - iiwa_joint_2
    - iiwa_joint_3
    - iiwa_joint_4
    - iiwa_joint_5
    - iiwa_joint_6
    - iiwa_joint_7
  end_effector: iiwa_link_ee            # Link to control arm in
  update_frequency: 500                 # Controller update frequency in Hz
  # Optional parameters - the mentioned values are the defaults
  verbose: false                        # Enables additional output
  dynamic_reconfigure: true             # Starts dynamic reconfigure server
  handle_trajectories: true             # Accept traj., e.g. from MoveIt
  robot_description: /robot_description # In case of a varying name
  from_frame_wrench: world              # Base frame of wrench msgs.
  to_frame_wrench: <end_effector>         # Frame for wrench commands
  delta_tau_max: 1.0                    # Max. commanded torque diff between steps
  filtering:                            # Update existing values (0.0 1.0] per s
    pose: 0.1                           # Reference pose filtering
    stiffness: 0.1                      # Cartesian and nullspace stiffness
    wrench: 0.1                         # Commanded torque
```

## Features

- Configurable stiffness values along all Cartesian dimensions
- Configurable damping factors along all Cartesian dimensions
- Change desired pose at runtime
- Apply Cartesian forces and torques at runtime
- Optional filtering of stiffnesses, pose and wrenches for smoother operation
- Can handle joint trajectories with nullspace configurations, e.g. from MoveIt
- Limit jerk

![](./res/flowchart.png)

## Limitations

- Friction is not accounted for
- Stiffness and damping values along the Cartesian dimensions are uncoupled
- Only tested for the LBR iiwa model

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