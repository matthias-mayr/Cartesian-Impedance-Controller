float32 time

ThreeDimensionalQuantity position

ThreeDimensionalQuantity position_d_

FourDimensionalQuantity orientation

FourDimensionalQuantity orientation_d_

#commanded torque
JointsQuantity tau_d

#joint states
JointsQuantity q

#velocity states
JointsQuantity dq

# Stiffness values [x, y, z, a, b, c] for the cartesian impedance, x, y, z in [N/m], a, b, c in [Nm/rad]. 
# Precondition: 0.0 <= x, y, z <= 5000.0 and 0.0 <= a, b, c <= 300.0. 
CartesianQuantity cartesian_stiffness

# The stiffness value for null space [Nm/rad]. 
# Precondition: 0.0 <= value. 
float64 nullspace_stiffness

# Cartesian wrench
CartesianWrench cartesian_wrench

 # Cartesian error
ThreeDimensionalQuantity error_position
ThreeDimensionalQuantity error_rotation
