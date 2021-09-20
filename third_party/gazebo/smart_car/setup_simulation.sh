#!/bin/bash

# TODO: create virtual frame buffer to handle headless mode
Xvfb :1 -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3

# sudo chmod 777 /opt/source-code/feagi-simulate/Gazebo-ign/freenove_4wd_car_description
cd /opt/source-code/feagi-simulate/Gazebo-ign/freenove_4wd_car_description
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 launch freenove_4wd_car_description diff_drive.launch.py
