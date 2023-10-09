#!/bin/bash

source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
chmod a+x src/mycobot.py
ros2 run cobot_arm mycobot.py
