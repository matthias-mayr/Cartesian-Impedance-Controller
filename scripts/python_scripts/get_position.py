#! /usr/bin/python

import subprocess
import get_data
import sys
import time

if len(sys.argv) != 1:
    exit("[get pos]: Invalid number of inputs.")

pos=get_data.data_array("rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/position")

print pos[0], pos[1], pos[2]
