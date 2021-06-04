#! /usr/bin/python


import math
import subprocess
import sys,getopt #accept arguments'
import get_data

if len(sys.argv) != 2:
    exit("Invalid number of inputs")

cmd_="rostopic echo -n 1 /clock"
times=get_data.data_array(cmd_)
secs=times[0]
deci_secs=times[1]/(10**9)
current_time=secs+deci_secs
waiting_time=float(sys.argv[1]) + current_time
print "Waiting for ",float(sys.argv[1]),"sim-seconds."

while current_time < waiting_time:
    cmd_="rostopic echo -n 1 /clock"
    times=get_data.data_array(cmd_)
    secs=times[0]
    deci_secs=times[1]/(10**9)
    current_time=secs+deci_secs
print "Done waiting"
