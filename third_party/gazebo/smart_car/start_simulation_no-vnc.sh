#!/bin/bash
cd /opt/source-code/freenove_4wd_car_description/ && source /opt/ros/foxy/setup.bash && cd .. && sudo chmod 777 freenove_4wd_car_description/ && cd freenove_4wd_car_description/ && colcon build && source install/setup.bash
Xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3
(ros2 launch freenove_4wd_car_description freenove_smart_car.launch.py &) && (ros2 run freenove_4wd_car_description controller.py)
