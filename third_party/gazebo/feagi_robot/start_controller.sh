#!/bin/bash
Xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &
export DISPLAY=:1.0
export RENDER_ENGINE_VALUES=ogre2
export MESA_GL_VERSION_OVERRIDE=3.3
GAZEBO="ruby"
ROS2="ros2"

rm empty.sdf
for pid in $(ps -ef | grep "gazebo" | awk '{print $2}'); do kill $pid; done ##Ensure that no gazebo is running prior to launch gazebo
for pid in $(ps -ef | grep "freenove_smart_car.launch.py" | awk '{print $2}'); do kill $pid; done #Destroy xterm prior to launch gazebo

if pgrep -x "$ROS2" >/dev/null && pgrep -x "$GAZEBO" >/dev/null
then
    echo "$ROS2 and $GAZEBO are already running."
else
    cd /opt/source-code/feagi_robot/ && source /opt/ros/foxy/setup.bash && cd .. && sudo chmod 777 feagi_robot/ && cd feagi_robot/ && colcon build --symlink-install
    wait -n
    xterm -hold -e "cd /opt/source-code/feagi_robot/ && source install/setup.bash && ros2 launch feagi_robot freenove_smart_car.launch.py" &
    while [[ $WMC == '' ]]
    do
      WMC=$(wmctrl -l | grep Gazebo | awk '{print $1}')
    done && wmctrl -i -r $WMC -b add,fullscreen & ##This will make gazbeo full screen once gazebo appears
fi
cd /opt/source-code/feagi_robot
while [ ! -f /opt/source-code/feagi_robot/empty.sdf ]; do
  if xprop -name "Gazebo" | grep -q "_NET_WM_STATE_FULLSCREEN"; then
    :
  else
    WMC=$(wmctrl -l | grep Gazebo | awk '{print $1}')
    wmctrl -i -r $WMC -b add,fullscreen &
  fi
  echo "no file"
done
if [ -f /opt/source-code/feagi_robot/empty.sdf ]; then
  echo "FILE EXISTS!!"
  cp empty.sdf models/sdf/freenove_smart_car.sdf
  ./start_controller.sh
fi