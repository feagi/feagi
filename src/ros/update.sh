#!/usr/bin/env bash

cd ~/ros2_ws/
sudo rm -R ~/ros2_ws/install/ ~/ros2_ws/build/ ~/ros2_ws/log/
colcon build --packages-select py_topic
