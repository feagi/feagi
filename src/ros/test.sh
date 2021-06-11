#!/bin/bash

#sudo apt-get install xterm	
xterm*allowTitleOps: false
name="FEAGI ARDIUNO FOXY TOPIC" #easier to remember which to run on.
echo $name
for name in $name
do
if [[ "$name" == 'FEAGI' ]]; then
#	if [ -d "~/feagi-core/" ]
#	then
#		feagi_title="FEAGI"
#		xterm -hold -e "echo -e '\033]2;'$feagi_title'\007' && cd ~/feagi-core/ && source ./environName/bin/activate && cd src/ && python3 main.py" &
#	else
#		echo $
#	fi	
feagi_title="FEAGI"
xterm -hold -e "echo -e '\033]2;'$feagi_title'\007' && cd ~/feagi-core/ && source ./environName/bin/activate && cd src/ && python3 main.py" &
elif [[ "$name" == 'ARDIUNO' ]]; then
arduino_title="Arduino"
xterm -hold -e "echo -e '\033]2;'$arduino_title'\007' && cd ~/ros2_ws && source install/setup.bash && ros2 run py_topic sonar_sensor" &
elif [[ "$name" == 'FOXY' ]]; then
foxy_title="FOXY"
xterm -hold -e "echo -e '\033]2;'$foxy_title'\007' && cd ~/ros2_ws && source install/setup.bash && ros2 run py_topic sonar_reader1" &
fi
echo "bwuk"
sleep 3
done
