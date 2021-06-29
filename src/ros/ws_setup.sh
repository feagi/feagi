#!/usr/bin/env bash
source /opt/ros/foxy/setup.bash
mypath=`pwd`
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src || exit
ros2 pkg create --build-type ament_python py_topic
cp $mypath/ros_laser_scan.py $mypath/ros_teleop.py $mypath/HC_SR04_Foxy.py $mypath/Micro-ROS/micro_ros.py ~/feagi-core/src/arduino/py2arduino.py ~/ros2_ws/src/py_topic/py_topic/
sed '9i\  <buildtool_depend>ament_python</buildtool_depend>\n  <exec_depend>rclpy</exec_depend>\n  <exec_depend>geometry_msgs</exec_depend>' ~/ros2_ws/src/py_topic/package.xml > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/package.xml
sed '23i\             "ros_laser_scan = py_topic.ros_laser_scan:main",\n             "ros_teleop = py_topic.ros_teleop:main"' ~/ros2_ws/src/py_topic/setup.py > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/setup.py
cd ~/ros2_ws || exit\
cp $mypath/setup.pu ~/ros2_ws/src/py_topic/
colcon build
echo 'sourcing...'
source ~/ros2_ws/install/setup.bash
echo 'completed'
#gnome-terminal -- ros2 run py_topic ros_laser_scan
#gnome-terminal -- ros2 run py_topic ros_teleop
