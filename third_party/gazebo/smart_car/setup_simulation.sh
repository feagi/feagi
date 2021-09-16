#!/bin/bash

# TODO: create virtual frame buffer to handle headless mode
sudo chmod 777 /opt/source-code/feagi-simulate/Gazebo-ign/freenove_4wd_car_description
cd /opt/source-code/feagi-simulate/Gazebo-ign/freenove_4wd_car_description
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 launch freenove_4wd_car_description diff_drive.launch.py
