#!/bin/bash
DIRECTORY="$HOME/Arduino/libraries/micro_ros_arduino/"

if [ -d "$DIRECTORY" ]
then
	export PATH=$PATH:/home/ubuntu/arduino-cli/bin
	arduino-cli lib search newping
	arduino-cli lib install newping
	cd ~
	cd Arduino/libraries/
	git clone -b foxy https://github.com/micro-ROS/micro_ros_arduino.git
else
	echo "Micro ros exists"
fi

cd arduino-cli/
export PATH=$PATH:/home/ubuntu/arduino-cli/bin
arduino-cli board attach serial:///dev/ttyACM0 micro-ros_publisher
arduino-cli compile --port /dev/ttyACM0 micro-ros_publisher
arduino-cli upload --port /dev/ttyACM0 micro-ros_publisher
