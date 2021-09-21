#!/bin/bash

# to deploy upon connecting to VNC server in container:
# docker run -e OPENBOX_ARGS='--startup "/root/setup_simulation.sh"' -p 6080:80 newest

source /opt/ros/foxy/setup.bash
cd /opt/source-code/freenove_4wd_car_description/
colcon build
source install/setup.bash
ros2 launch freenove_4wd_car_description diff_drive.launch.py &
xterm -e ./start_controller.sh
