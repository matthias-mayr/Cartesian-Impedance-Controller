# Cartesian Stiffness values for translation and rotation of the axes x,y,z
# Values: Translation: [0.0, 2000.0] in [N/m]; Rotation [0.0; 500.0] in [Nm/rad]
geometry_msgs/Wrench cartesian_stiffness

# Damping factor for translation and rotation of the axes x,y,z
# The rule is always 2*sqrt(stiffness)
# Values: [0.001, 2.0]
geometry_msgs/Wrench cartesian_damping_factors

# Stiffness value for nullspace
# Values: >= 0.0 in [Nm/rad]
float64 nullspace_stiffness

# Damping parameter for nullspace [Nm*s/rad]. 
# Value: [0.001, 2.0]
float64 nullspace_damping_factor

# The desired nullspace configuration
# Values: According to the limits of the robot
float64[] q_d_nullspace