#!/bin/bash
echo "First time on the board only."
echo "This will erase everything in your board."
echo "This will start in 5 seconds. "
echo "To cancel this process, press ctrl C"
sleep 5

/bin/bash ./sonar_setup.sh
export PATH=$PATH:/root/$USER/arduino-cli/bin
mkdir ~/arduino-cli/ardiunotopython/
cp ~/ardiunotopython.ino ~/arduino-cli/ardiunotopython/
arduino-cli core install arduino:avr
arduino-cli lib search newping
arduino-cli lib install newping
cd arduino-cli/
export PATH=$PATH:/root/$USER/arduino-cli/bin
arduino-cli board attach serial:///dev/ttyUSB0 ardiunotopython
arduino-cli compile --port /dev/ttyUSB0 ardiunotopython
arduino-cli upload --port /dev/ttyUSB0 ardiunotopython

source /opt/ros/foxy/setup.bash
cd ~/ros2_ws && source install/setup.bash && ros2 run py_topic sonar_sensor
