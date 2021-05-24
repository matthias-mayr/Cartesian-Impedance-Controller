#! /usr/bin/python

import math
import subprocess
import re

#Define commands that recieves data
cmd_pos='rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/position'
cmd_pos_d='rostopic echo -n 1 /bh/CartesianImpedance_trajectory_controller/useful_data_to_analyze/position_d_'

#Retrieve data from output of commands
pos_prc= subprocess.Popen(cmd_pos,shell=True,stdout=subprocess.PIPE) 
pos_d_prc= subprocess.Popen(cmd_pos_d,shell=True,stdout=subprocess.PIPE) 

#Extract data string
pos = pos_prc.communicate()[0]
pos_d = pos_d_prc.communicate()[0]

#Split data to get the values
data_pos=pos.split()
data_pos_d=pos_d.split()

#Declare values
x=float(data_pos[1])
y=float(data_pos[3])
z=float(data_pos[5])

x_d=float(data_pos_d[1])
y_d=float(data_pos_d[3])
z_d=float(data_pos_d[5])

dist= math.sqrt((x-x_d)**2+(y-y_d)**2+(z-z_d)**2)

print dist


#Perform calculations




