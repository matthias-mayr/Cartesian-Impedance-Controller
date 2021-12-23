Header header

geometry_msgs/Pose current_pose
geometry_msgs/Pose reference_pose
geometry_msgs/Pose pose_error

sensor_msgs/JointState joint_state  # Effort are commanded efforts

# Stiffness values for the cartesian impedance, x, y, z in [N/m], x, y, z in [Nm/rad]. 
geometry_msgs/Wrench cartesian_stiffness

# The stiffness value for null space [Nm/rad]. 
# Precondition: 0.0 <= value. 
float64 nullspace_stiffness

geometry_msgs/Wrench commanded_wrench

float64 cartesian_velocity              # Calculated Cartesian velocity
