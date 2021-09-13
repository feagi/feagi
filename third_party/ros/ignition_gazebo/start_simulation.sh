#!/bin/bash

source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
# ros2 launch freenove_4wd_car_description diff_drive.launch.py
# ros2 run freenove_4wd_car_description range.py
