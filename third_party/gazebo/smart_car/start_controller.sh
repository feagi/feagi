#!/bin/bash
Xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3
GAZEBO="ruby"
ROS2="ros2"

if pgrep -x "$ROS2" >/dev/null && pgrep -x "$GAZEBO" >/dev/null
then
    echo "$ROS2 and $GAZEBO are already running."
else
    cd /opt/source-code/freenove_4wd_car_description/ && source /opt/ros/foxy/setup.bash && cd .. && sudo chmod 777 freenove_4wd_car_description/ && cd freenove_4wd_car_description/ && colcon build --symlink-install &
    wait -n
    sleep 7 && wmctrl -r Gazebo -b toggle,fullscreen
    xterm -hold -e "cd /opt/source-code/freenove_4wd_car_description/ && source install/setup.bash && ros2 launch freenove_4wd_car_description freenove_smart_car.launch.py"

fi