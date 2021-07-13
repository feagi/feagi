#!/bin/bash

#Install micro-ros
# cd ~
# mkdir micro_ros_arduino
# cd ~/micro_ros_arduino
# source /opt/ros/$ROS_DISTRO/setup.bash
# git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# rosdep update && rosdep install --from-path src --ignore-src -y
# cd ~/micro_ros_arduino || exit
# colcon build
# source install/local_setup.bash
# ros2 run micro_ros_setup create_agent_ws.sh
# ros2 run micro_ros_setup build_agent.sh
# source install/local_setup.sh


#Install Arduino CLI
cd ~
pip3 install pyserial
git clone https://github.com/arduino/arduino-cli.git
cd arduino-cli/
export PATH=$PATH:/root/$USER/arduino-cli/bin
./install.sh
export PATH=$PATH:/root/$USER/arduino-cli/bin
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:samd
arduino-cli core install arduino:sam
arduino-cli core install arduino:avr
# mkdir micro-ros_publisher
# cd micro-ros_publisher
# cp ~/micro-ros_publisher.ino ~/arduino-cli/micro-ros_publisher/
# cd ~/.arduino15/packages/arduino/hardware/sam/1.6.12/
# curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_arduinocore_sam.txt > platform.txt