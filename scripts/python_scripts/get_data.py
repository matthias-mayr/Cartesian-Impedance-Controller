#! /usr/bin/python
#######################
import math
import subprocess
import re
import sys,getopt #accept arguments


def data_array(cmd_):

    #Retrieve data from output of commands
    prc_ = subprocess.Popen(cmd_,shell=True,stdout=subprocess.PIPE) 


    #Extract data string
    string_ = prc_.communicate()[0]


    #Split data to get the values
    string_array= string_.split()
    data=[]
    for possible_float_ in string_array:
        try:
            data.append(float (possible_float_.strip()))
        except:
            pass
    return data








