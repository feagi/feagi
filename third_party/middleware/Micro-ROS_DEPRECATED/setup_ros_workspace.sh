#!/bin/bash

cd ~
mypath=`pwd`
pip3 install zmq
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src || exit
ros2 pkg create --build-type ament_python py_topic
sudo cp -R $mypath/ros_laser_scan.py $mypath/ros_teleop.py ~/ros2_ws/src/py_topic/py_topic/
sed '9i\  <buildtool_depend>ament_python</buildtool_depend>\n  <exec_depend>rclpy</exec_depend>\n  <exec_depend>geometry_msgs</exec_depend>' ~/ros2_ws/src/py_topic/package.xml > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/package.xml
sed '23i\             "ros_laser_scan = py_topic.ros_laser_scan:main",\n             "ros_teleop = py_topic.ros_teleop:main"' ~/ros2_ws/src/py_topic/setup.py > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/setup.py
cp ~/setup.py ~/ros2_ws/src/py_topic/
cd ~/ros2_ws/ || exit
colcon build
source /opt/ros/foxy/setup.bash
