#!/bin/bash
GAZEBO="ruby"
ROS2="ros2"

if pgrep -x "$ROS2" >/dev/null
then
    echo "$ROS2 is already running"
else
    cd /opt/source-code/freenove_4wd_car_description/ && source /opt/ros/foxy/setup.bash && cd .. && sudo chmod 777 freenove_4wd_car_description/ && cd freenove_4wd_car_description/ && colcon build --symlink-install &
    wait -n
    xterm -hold -e "cd /opt/source-code/freenove_4wd_car_description/ && source install/setup.bash && ros2 run freenove_4wd_car_description controller.py" &
    sleep 7 && wmctrl -r Gazebo -b toggle,fullscreen &
fi

if pgrep -x "$GAZEBO" >/dev/null
then
    echo "$GAZEBO is already running"
else
    cd /opt/source-code/freenove_4wd_car_description/ && source install/setup.bash && ros2 launch freenove_4wd_car_description freenove_smart_car.launch.py
fi 
