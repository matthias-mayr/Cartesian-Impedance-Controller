#!/bin/bash

echo "Returning to the \"home\" position";
./change_nullspace_config.sh 1.2701715984878614 0.06980988466741955 0.2716601287156033 -0.9147682346772754 0.3409470977823119 1.175819569186606 0.69816013366456 & ./change_goal.sh -0.5 -0.5 1 1 0 0 0 & ./change_stiffness.sh 50 50 50 30 30 30 20

python python_scripts/wait_for_stiffness.py 50 50 50 30 30 30 20

./change_stiffness.sh 200 200 200 100 100 100 0


python python_scripts/wait_for_stiffness.py 200 200 200 100 100 100 0

