#!/bin/bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
ros2 run py_topic ros_laser_scan
# ros2 run turtlebot3_teleop teleop_keyboard
