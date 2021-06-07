#! /usr/bin/python

import subprocess
import get_data
import sys
import time


timeout = time.time() + 80   # 60 seconds from now
while time.time() < timeout:
    cartesian_velocity=get_data.data_array('rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/cartesian_velocity')
    ready=True
    if abs(cartesian_velocity[0]) > 0.0005:
        ready = False
    if ready == True:
        exit("Robot is not moving anymore")
    else:
        print 'Robot still in motion. Time left:', timeout-time.time()

