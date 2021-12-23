# Cartesian Stiffness values for translation and rotation of the axes x,y,z
# Values: Translation: [0.0, 2000.0] in [N/m]; Rotation [0.0; 500.0] in [Nm/rad]
geometry_msgs/Wrench cartesian_stiffness

# Damping values for translation and rotation of the axes x,y,z
# Values: [0.0, 1.0]
geometry_msgs/Wrench cartesian_damping

# Stiffness value for nullspace
# Values: >= 0.0 in [Nm/rad]
float64 nullspace_stiffness

# Damping parameter for nullspace [Nm*s/rad]. 
# Value: [0.3, 1.0]; A good damping value is 0.7. 
float64 nullspace_damping

# The desired nullspace configuration
# Values: According to the limits of the robot
float64[] q_d_nullspace