#!/bin/bash

sleep 10

./change_stiffness.sh 50 50 50 10 10 10 0

sleep 1
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0

sleep 20

./change_goal.sh -0.5 0.0 1 0.707 0.0 0.0 -0.707

sleep 20

./change_goal.sh -0.45 -0.5 1.1 1.0 0.0 0.0 0.0

sleep 20
./home_position.sh

echo "Motion complete"
