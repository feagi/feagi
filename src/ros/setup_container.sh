#!/bin/bash

# turtlebot3 setup
cd ~
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
sudo chmod 755 ./install_ros2_foxy.sh
bash ./install_ros2_foxy.sh
sudo apt install -y libpython3-dev python3-pip
pip3 install -U argcomplete
sudo apt-get install -y ros-foxy-gazebo-*
sudo apt install -y ros-foxy-cartographer
sudo apt install -y  ros-foxy-cartographer-ros
sudo apt install -y  ros-foxy-navigation2
sudo apt install -y  ros-foxy-nav2-bringup
sudo apt install -y  ros-foxy-dynamixel-sdk
sudo apt install -y  ros-foxy-turtlebot3
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws/ || exit
source /opt/ros/foxy/setup.bash
colcon build
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc


# Install Terminator
sudo add-apt-repository ppa:gnome-terminator
sudo apt-get update
sudo apt-get -y  install terminator

#Install arduino
cd ~
wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
tar -xf arduino-1.8.13-linux64.tar.xz

# ros workspace setup
cd ~
mypath=`pwd`
pip3 install zmq
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src || exit
ros2 pkg create --build-type ament_python py_topic
sudo cp -R $mypath/ros_laser_scan.py $mypath/HC_SR04_Foxy.py $mypath/Sonar_reader1.py $mypath/Sonar_reader.py $mypath/micro-ros_publisher/ $mypath/ardiunotopython/ $mypath/ros_teleop.py ~/ros2_ws/src/py_topic/py_topic/
sed '9i\  <buildtool_depend>ament_python</buildtool_depend>\n  <exec_depend>rclpy</exec_depend>\n  <exec_depend>geometry_msgs</exec_depend>' ~/ros2_ws/src/py_topic/package.xml > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/package.xml
sed '23i\             "ros_laser_scan = py_topic.ros_laser_scan:main",\n             "ros_teleop = py_topic.ros_teleop:main"' ~/ros2_ws/src/py_topic/setup.py > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/setup.py
cp ~/setup.py ~/ros2_ws/src/py_topic/
cd ~/ros2_ws/ || exit

pip3 install zmq
pip3 install pyserial
colcon build
source /opt/ros/foxy/setup.bash

#Install micro-ros
cd ~
mkdir micro_ros_arduino
cd ~/micro_ros_arduino
source /opt/ros/$ROS_DISTRO/setup.bash
git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-path src --ignore-src -y
cd ~/micro_ros_arduino || exit
sudo rm -R log/ build/ install/
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh

#Install Arduino CLI
cd ~
git clone https://github.com/arduino/arduino-cli.git
cd arduino-cli/
./install.sh
export PATH=$PATH:/home/ubuntu/arduino-cli/bin
arduino-cli config init
arduino-cli core update-index
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:samd:arduino_zero_edbg micro-ros_publisher

