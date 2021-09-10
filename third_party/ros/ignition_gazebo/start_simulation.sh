#!/bin/bash

source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch freenove_4wd_car_description diff_drive.launch.py
