#!/usr/bin/env bash
source /opt/ros/foxy/setup.bash
mypath=`pwd`
cd ~/ros2_ws/
cp $mypath/HC_SR04_Foxy.py ~/ros2_ws/src/py_topic/py_topic/
cp $mypath/setup.py ~/ros2_ws/src/py_topic/
colcon build
source ~/ros2_ws/install/setup.bash
echo "completed"
