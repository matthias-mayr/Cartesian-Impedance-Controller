#!/bin/bash

./change_stiffness.sh 200 200 200 100 100 100 0

sleep 1
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0

sleep 20

./change_goal.sh -0.5 0.0 1 0.707 0.0 0.0 -0.707

sleep 20

./change_goal.sh -0.45 -0.5 1.1 1.0 0.0 0.0 0.0

sleep 20
./home_position.sh

echo "Motion complete"
