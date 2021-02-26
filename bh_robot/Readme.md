# Robot Descriptions and Bringup

This repository has been inherited from RVMI Copenhagen. We haven't renamed everything, because it would be quite a lot of work and make things less compatible.

## Start
Start the whole setup in simulation with
```
mon launch bh_robot bringup.launch
```
If you are using the real robot, launch `core.launch` first to set the parameters for the tools. That would be
```
mon launch bh_robot core.launch
mon launch bh_robot bringup.launch sim:=false
```
After that launch the sunrise programs on the robots.
### Parameters to launch files
The launch file `bringup.launch` has several parameters. The most used should be `run_moveit`, `world` and `sim`. `world` sets a gazebo world defined in `bh_gazebo/worlds/`.


## Name Correspondences

| What           | RVMI Name | RVMI Alternative Name | Our Name |  Our Alternative Name |
|----------------|-----------|-----------------------|----------|-----------------------|
| Dual Arm Setup | bh        | dual_arm              | bh       | dual_arm              |
| Left Arm       | bh        |                       | munin    | bh (for now)          |
| Left Gripper   | psa       | piston                | (none)   | psa in Sunrise        |
| Right Arm      | hh        |                       | hugin    | hh (for now)          |
| Right Gripper  | wsg       | schunk                | robotiq  | wsg in Sunrise        |


## Other things to know
* `wsg` is the Schunk WSG 50 two finger electric gripper
* `psa`/`piston` is a special peumatic gripper for pistons
* This will start the camera, but the transformation between gripper and camera is expected to be published externally. Therefore you will receive tf warnings if you only start this launch file.


## Structure

```
├── bh_control              Startup and config for controllers and hardware
│   ├── config                  Control values and joint names
│   ├── launch                  Launch files included by bh_robot
│   └── test                    Sends a test trajectory
├── bh_description          CAD, URDF and launch files
│   ├── calibration             Some values, not really used
│   ├── config                  jsp configuration
│   ├── launch                  Description launch files
│   ├── meshes                  Meshes for hardware
│   │   ├── iiwa_mount              Our aluminium toros
│   │   ├── piston_gripper          PSA/Piston gripper (currently not used)
│   │   ├── primesense              Camera
│   │   └── schunk_wsg50            Schunk WSG 2 finger gripper
│   ├── robots                      Our robot URDFs
│   └── urdf                   URDF resources
│       ├── arms                    Generic iiwa arms
│       ├── end_effectors           psa/wsg gripper
│       ├── iiwa_mount              Our aluminium torso
│       └── primesense              Camera
├── bh_gazebo               Gazebo configuration
│   ├── include                Floating point plugin
│   ├── launch                 Gazebo launch file
│   ├── src                    Floating point plugin
│   └── worlds                 World definitions
├── bh_moveit               MoveIt configuration
│   ├── config
│   └── launch
└── bh_robot                Actual launch files that are called
    ├── config                  rviz config
    ├── launch                  bringup launch file
    └── scripts                 AAU home position script
```

