#!/usr/bin/env bash
mkdir -p ./ros2_ws/src
cd ./ros2_ws/src
ros2 pkg create --build-type ament_python py_topic
cp ./ros* ./ros2_ws/src/py_topic/py_topic/
sed '9i\  <buildtool_depend>ament_python</buildtool_depend>' ./ros2_ws/src/py_topic/package.xml
sed '9i\  <exec_depend>rclpy</exec_depend>' ./ros2_ws/src/py_topic/package.xml
sed '9i\  <exec_depend>geometry_msgs</exec_depend>' ./ros2_ws/src/py_topic/package.xml
