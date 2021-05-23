echo "This experimental motion consists of some trajectories with intermidiate pauses."

./home_position.sh

echo "Press any key to start"
read step1

./motion1.sh & ./log_data.sh 40 motion1


