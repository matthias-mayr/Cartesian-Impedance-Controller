#! /usr/bin/python

import math
import subprocess
import re

#Define commands that recieves data
cmd_pos_err='rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/error_position'
cmd_rot_err='rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/error_rotation'

#Retrieve data from output of commands
prc_pos_err= subprocess.Popen(cmd_pos_err,shell=True,stdout=subprocess.PIPE) 
prc_rot_err= subprocess.Popen(cmd_rot_err,shell=True,stdout=subprocess.PIPE) 

#Extract data string
pos_err = prc_pos_err.communicate()[0]
rot_err = prc_rot_err.communicate()[0]

#Split data to get the values
data_pos_err=pos_err.split()
data_rot_err=rot_err.split()

#Declare values
e_pos_x=float(data_pos_err[1])
e_pos_y=float(data_pos_err[3])
e_pos_z=float(data_pos_err[5])

e_rot_x=float(data_rot_err[1])
e_rot_y=float(data_rot_err[3])
e_rot_z=float(data_rot_err[5])

pos_distance= math.sqrt((e_pos_x)**2+(e_pos_y)**2+(e_pos_z)**2)
rot_distance=math.sqrt((e_rot_x)**2+(e_rot_y)**2+(e_rot_z)**2)

print int(pos_distance<0.005) and int(rot_distance<0.005)





