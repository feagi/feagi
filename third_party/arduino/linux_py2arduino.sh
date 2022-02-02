#!/bin/bash

sudo apt-get install xterm	
#xterm*allowTitleOps: false
name="FEAGI ARDIUNO FOXY TOPIC" #easier to remember which to run on.
check="xterm"
DIRECTORY="$HOME/feagi/"
dpkg -s $check &> /dev/null  
if [ $check -ne 0 ]
then
	echo "not installed"  
	sudo apt-get update
	sudo apt-get install $check

else
	echo    "installed"
fi


echo $name

for name in $name
do
if [[ "$name" == 'FEAGI' ]]; then
	if [ -d "$DIRECTORY" ]
	then
		feagi_title="FEAGI"
		xterm -hold -e "echo -e '\033]2;'$feagi_title'\007' && cd ~/feagi/ && source ./environName/bin/activate && cd src/ && python3 main.py" &
	else
		echo "no FEAGI directory"
		echo $DIRECTORY
	fi	
#feagi_title="FEAGI"
#xterm -hold -e "echo -e '\033]2;'$feagi_title'\007' && cd ~/feagi/ && source ./environName/bin/activate && cd src/ && python3 main.py" &
elif [[ "$name" == 'ARDIUNO' ]]; then
arduino_title="Arduino"
xterm -hold -e "echo -e '\033]2;'$arduino_title'\007' && cd ~/ros2_ws && source install/setup.bash && ros2 run py_topic sonar_sensor" &
elif [[ "$name" == 'FOXY' ]]; then
foxy_title="FOXY"
xterm -hold -e "echo -e '\033]2;'$foxy_title'\007' && cd ~/ros2_ws && source install/setup.bash && ros2 run py_topic py2arduino" &
fi
sleep 3
done
