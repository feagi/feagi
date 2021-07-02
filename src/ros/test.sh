#!/bin/bash
echo "First time on the board only."
echo "This will erase everything in your board."
echo "This will start in 5 seconds. "
echo "To cancel this process, press ctrl C"
sleep 5
export PATH=$PATH:/home/$USER/arduino-cli/bin
mkdir ~/arduino-cli/ardiunotopython/
mv ~/feagi-core/src/arduino/ardiunotopython/ardiunotopython.ino ~/arduino-cli/ardiunotopython/
arduino-cli core install arduino:avr
arduino-cli lib search newping
arduino-cli lib install newping
cd ~
cd Arduino/libraries/
git clone -b foxy https://github.com/micro-ROS/micro_ros_arduino.git
cd ~
cd arduino-cli/
export PATH=$PATH:/home/$USER/arduino-cli/bin
arduino-cli board attach serial:///dev/ttyACM0 ardiunotopython
arduino-cli compile --port /dev/ttyACM0 ardiunotopython
arduino-cli upload --port /dev/ttyACM0 ardiunotopython
#/bin/bash ~/linux_py2arduino.sh
cd ~/ros2_ws && source install/setup.bash && ros2 run py_topic sonar_sensor
