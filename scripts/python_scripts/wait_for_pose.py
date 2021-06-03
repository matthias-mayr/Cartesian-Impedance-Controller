#! /usr/bin/python

import subprocess
import get_data
import sys
import time



timeout = time.time() + 60   # 60 seconds from now
while time.time() < timeout:
    cmd_=subprocess.Popen('./check_pose.py',shell=True,stdout=subprocess.PIPE)
    is_near=cmd_.communicate[0]
    print is_near



