#!/bin/bash
Xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3
GAZEBO="ruby"
ROS2="ros2"
cd /lib/x86_64-linux-gnu/ign-rendering-3/engine-plugins && export IGN_GAZEBO_RENDER_ENGINE_PATH=$PWD

if pgrep -x "$ROS2" >/dev/null && pgrep -x "$GAZEBO" >/dev/null
then
    echo "$ROS2 and $GAZEBO are already running."
else
    cd /opt/source-code/gazebo/ && source /opt/ros/foxy/setup.bash && cd .. && sudo chmod 777 gazebo/ && cd gazebo/ && colcon build --symlink-install &
    wait -n
    sleep 7 && wmctrl -r Gazebo -b toggle,fullscreen
    xterm -hold -e "cd /opt/source-code/gazebo/ && source install/setup.bash && ros2 launch gazebo low_res_freenove_smart_car.launch.py"

fi
