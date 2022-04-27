#!/bin/bash
cd /opt/source-code/freenove_4wd_car_description/ && source /opt/ros/foxy/setup.bash && cd .. && sudo chmod 777 freenove_4wd_car_description/ && cd freenove_4wd_car_description/ && colcon build --symlink-install &
wait -n
xterm -hold -e "cd /opt/source-code/freenove_4wd_car_description/ && source install/setup.bash && ros2 run freenove_4wd_car_description controller.py" & 
sleep 7 && wmctrl -r Gazebo -b toggle,fullscreen &
cd /opt/source-code/freenove_4wd_car_description/ && source install/setup.bash && ros2 launch freenove_4wd_car_description freenove_smart_car.launch.py
