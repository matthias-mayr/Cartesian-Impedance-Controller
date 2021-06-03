#! /usr/bin/python

import subprocess
import get_data


#subprocess.call("./get_data.py 'rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/cartesian_stiffness'", shell=True)
#output=subprocess.check_output("./get_data.py 'rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/cartesian_stiffness'", shell=True)
cartesian_stiffness=get_data.data_array('rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/cartesian_stiffness')

print cartesian_stiffness



