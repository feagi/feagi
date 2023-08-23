#!/bin/bash
GAZEBO="ruby"
ROS2="ros2"

rm empty.sdf
rm new.txt
for pid in $(ps -ef | grep "gazebo" | awk '{print $2}'); do kill $pid; done ##Ensure that no gazebo is running prior to launch gazebo
for pid in $(ps -ef | grep "robot.launch.py" | awk '{print $2}'); do kill $pid; done #Destroy xterm prior to launch gazebo
for pid in $(ps -ef | grep "xterm " | awk '{print $2}'); do kill $pid; done
for pid in $(ps -ef | grep "parameter_bridg" | awk '{print $2}'); do kill $pid; done
for pid in $(ps -ef | grep "robot.py" | awk '{print $2}'); do kill $pid; done


if pgrep -x "$ROS2" >/dev/null && pgrep -x "$GAZEBO" >/dev/null
then
    for pid in $(ps -ef | grep "gazebo" | awk '{print $2}'); do kill $pid; done
    # Ensure that no gazebo is running prior to launch gazebo
    for pid in $(ps -ef | grep "robot.launch.py" | awk '{print $2}'); do kill $pid; done
    #Destroy xterm prior to launch gazebo
    for pid in $(ps -ef | grep "xterm " | awk '{print $2}'); do kill $pid; done
    for pid in $(ps -ef | grep "parameter_bridg" | awk '{print $2}'); do kill $pid; done
    for pid in $(ps -ef | grep "robot.py" | awk '{print $2}'); do kill $pid; done
fi
cd /opt/source-code/simulation/
if [ ! -d "install" ]; then
  source /opt/ros/foxy/setup.bash && cd .. && sudo chmod 777 simulation/ && cd simulation/ && colcon build --symlink-install
  source install/setup.bash
else
  source /opt/ros/foxy/setup.bash && source install/setup.bash
fi
wait -n
#    xterm -fa "Terminus" -fs 6 -hold -e "ros2 launch simulation robot.launch.py" &
ros2 launch simulation robot.launch.py &
while [[ $WMC == '' ]]
do
  WMC=$(wmctrl -l | grep Gazebo | awk '{print $1}')
done && wmctrl -i -r $WMC -b add,fullscreen & ##This will make gazbeo full screen once gazebo appears
cd /opt/source-code/simulation
while [ ! -f /opt/source-code/simulation/empty.sdf ] && [ ! -f /opt/source-code/simulation/new.txt ]; do
  :
  if xprop -name "Gazebo" >/dev/null 2>&1 | grep -q "_NET_WM_STATE_FULLSCREEN" >/dev/null 2>&1; then
    :
  else
    WMC=$(wmctrl -l | grep Gazebo | awk '{print $1}')
    wmctrl -i -r $WMC -b add,fullscreen &
  fi
  #echo "no file"
done
if [ -f /opt/source-code/simulation/empty.sdf ]; then
  cp empty.sdf robots/freenove_smart_car.sdf
fi
if [ -f /opt/source-code/simulation/new.txt ]; then
  rm new.txt
fi
./start_controller.sh &
echo "EXITED"