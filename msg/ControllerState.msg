Header header

geometry_msgs/Pose current_pose
geometry_msgs/Pose reference_pose
geometry_msgs/Pose pose_error

sensor_msgs/JointState joint_state
float64[] commanded_torques

# Stiffness values for the cartesian impedance, x, y, z in [N/m], x, y, z in [Nm/rad]. 
geometry_msgs/Wrench cartesian_stiffness
geometry_msgs/Wrench cartesian_damping

float64 nullspace_stiffness
float64 nullspace_damping
float64[] nullspace_config

geometry_msgs/Wrench commanded_wrench

float64 cartesian_velocity              # Calculated Cartesian velocity [m/s]