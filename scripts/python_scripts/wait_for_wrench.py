#! /usr/bin/python

import subprocess
import get_data
import sys
import time

if len(sys.argv) != 7:
    exit("Invalid number of inputs")

timeout = time.time() + 10   # 60 seconds from now
while time.time() < timeout:
    cartesian_wrench=get_data.data_array('rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/cartesian_wrench')
    ready=True
    for i in range(1, len(sys.argv)):
        if  abs(float(sys.argv[i])+cartesian_wrench[i-1])  > 0.1:
            ready = False
    if ready == True:
        exit("Wrench ready")
    else:
        print 'Waiting for wrench. Time left:', timeout-time.time()

