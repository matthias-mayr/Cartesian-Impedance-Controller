#! /usr/bin/python

import subprocess
import get_data
import sys
import time

if len(sys.argv) != 2:
    exit("[is_below_height]: Invalid number of inputs.")

current_height=height=get_data.data_array("rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/position/z")

print int(float(current_height[0]) > float(sys.argv[1]))


