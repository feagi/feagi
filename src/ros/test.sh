#!/bin/bash
echo "First time on the board only."
echo "This will erase everything in your board."
echo "This will start in 5 seconds. "
echo "To cancel this process, press ctrl C"
sleep 5
export PATH=$PATH:/home/ubuntu/arduino-cli/bin
mkdir ~/arduino-cli/ardiunotopython/
mv ardiunotopython.ino ~/arduino-cli/ardiunotopython/
arduino-cli core install arduino:avr
arduino-cli lib search newping
arduino-cli lib install newping
cd ~
cd Arduino/libraries/
git clone -b foxy https://github.com/micro-ROS/micro_ros_arduino.git
cd ~
cd arduino-cli/
export PATH=$PATH:/home/ubuntu/arduino-cli/bin
arduino-cli board attach serial:///dev/cu.usbserial-1430 ardiunotopython
arduino-cli compile --port /dev/cu.usbserial-1430 ardiunotopython
arduino-cli upload --port /dev/cu.usbserial-1430 ardiunotopython
#/bin/bash ~/linux_py2arduino.sh
