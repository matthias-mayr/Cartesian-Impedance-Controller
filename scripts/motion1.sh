#!/bin/bash

echo "This experimental motion consists of some trajectories with intermidiate pauses."

./home_position.sh

echo "Press any key to start"
read step1
./change_goal.sh -0.5 0.0 1 1.0 0.0 0.0 0.0


echo "Press any key to continue to the next trajectory"
read step2
./change_goal.sh -0.5 0.0 1 0.707 0.0 0.0 -0.707

echo "Press any key to continue to the next trajectory"
read step3
./change_goal.sh -0.45 -0.5 1.1 1.0 0.0 0.0 0.0


echo "Press any key to go back to home position"
read step4
./home_position.sh

echo "Motion complete"
