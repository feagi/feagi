#!/bin/bash

#sudo apt-get install xterm	
name="FEAGI FOXY ARDIUNO TOPIC" #easier to remember which to run on.
echo $name
for name in $name
do
if [[ "$name" == 'FEAGI' ]]; then
xterm -hold -e "cd ~/ros2_ws && source install/setup.bash && echo done && ros2 run py_topic sonar_sensor;bash" &
fi
echo "bwuk"
sleep 2
done
