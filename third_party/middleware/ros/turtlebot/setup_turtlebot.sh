#!/bin/bash

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws || exit
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/foxy-devel/turtlebot3.repos
vcs import ~/turtlebot3_ws/src < turtlebot3.repos
source /opt/ros/foxy/setup.bash
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --continue-on-error
source install/setup.bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc