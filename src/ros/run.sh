#!/bin/bash
source /opt/ros/foxy/setup.bash
xterm -hold ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# xterm ros2 run py_topic py_laser_scan
# xterm ros2 run turtlebot3_teleop teleop_keyboard