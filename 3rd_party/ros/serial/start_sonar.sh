#!/bin/bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws && source install/setup.bash && ros2 run py_topic sonar_sensor
