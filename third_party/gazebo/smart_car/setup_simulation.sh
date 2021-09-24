#!/bin/bash

# to deploy upon connecting to VNC server in container:
# docker run -e OPENBOX_ARGS='--startup "/root/setup_simulation.sh"' -p 6080:80 newest

sudo chmod 777 /opt/source-code/freenove_4wd_car_description
source /opt/ros/foxy/setup.bash
cd /opt/source-code/freenove_4wd_car_description/
colcon build
source install/setup.bash
ros2 launch freenove_4wd_car_description freenove_smart_car.launch.py &
xterm -hold -e "echo -e '\033]2;'$foxy_title'\007' && cd /opt/source-code/freenove_4wd_car_description/ && bash ./start_ultrasonic.sh" &
