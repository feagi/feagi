#!/bin/bash
echo "First time on the board only."
echo "This will erase everything in your board."
echo "This will start in 5 seconds. "
echo "To cancel this process, press ctrl C"
sleep 5
export PATH=$PATH:/home/ubuntu/arduino-cli/bin
arduino-cli lib search newping
arduino-cli lib install newping
cd ~
cd Arduino/libraries/
git clone -b foxy https://github.com/micro-ROS/micro_ros_arduino.git
cd ~
cd arduino-cli/
export PATH=$PATH:/home/ubuntu/arduino-cli/bin
arduino-cli board attach serial:///dev/ttyACM0 micro-ros_publisher
arduino-cli compile --port /dev/ttyACM0 micro-ros_publisher
arduino-cli upload --port /dev/ttyACM0 micro-ros_publisher
