#!/bin/bash

# turtlebot3 setup
mkdir -p ~/turtlebot3_ws/src
cd turtlebot3_ws || exit
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import ~/turtlebot3_ws/src < turtlebot3.repos
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
echo "source /install/setup.bash" >> /root/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> /root/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> /root/.bashrc
echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
source /root/.bashrc

# ros workspace setup
cd ~
mypath=`pwd`
pip3 install zmq
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src || exit
ros2 pkg create --build-type ament_python py_topic
cp $mypath/ros_laser_scan.py $mypath/ros_teleop.py ~/ros2_ws/src/py_topic/py_topic/
sed '9i\  <buildtool_depend>ament_python</buildtool_depend>\n  <exec_depend>rclpy</exec_depend>\n  <exec_depend>geometry_msgs</exec_depend>' ~/ros2_ws/src/py_topic/package.xml > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/package.xml
sed '23i\             "ros_laser_scan = py_topic.ros_laser_scan:main",\n             "ros_teleop = py_topic.ros_teleop:main"' ~/ros2_ws/src/py_topic/setup.py > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/setup.py
cd ~/ros2_ws || exit
colcon build
source ~/ros2_ws/install/setup.bash

# run ros processes
# gnome-terminal ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# gnome-terminal ros2 run py_topic py_laser_scan
# gnome-terminal ros2 run turtlebot3_teleop teleop_keyboard