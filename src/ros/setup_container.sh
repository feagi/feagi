#!/bin/bash

# turtlebot3 setup
cd ~
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
sudo chmod 755 ./install_ros2_foxy.sh
bash ./install_ros2_foxy.sh
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
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

# FEAGI setup
cd ~
git clone git@github.com:feagi/feagi-core.git
cd feagi-core/
pip3 install virtualenv
virtualenv -p /usr/bin/python3 environName
source ./environName/bin/activate
pip3 install -r requirements.txt
sudo mkdir /mnt/ramdisk
sudo mount -t tmpfs -o rw,size=1000M tmpfs /mnt/ramdisk
python3 ./src/cython_libs/cython_setup.py build_ext --inplace
sudo wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list
apt-get update
apt-get install -y mongodb-org
apt-get install libatlas-base-dev

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
cp $mypath/ros_laser_scan.py $mypath/ros_teleop.py ~/ros2_ws/src/py_topic/py_topic/
sed '9i\  <buildtool_depend>ament_python</buildtool_depend>\n  <exec_depend>rclpy</exec_depend>\n  <exec_depend>geometry_msgs</exec_depend>' ~/ros2_ws/src/py_topic/package.xml > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/package.xml
sed '23i\             "ros_laser_scan = py_topic.ros_laser_scan:main",\n             "ros_teleop = py_topic.ros_teleop:main"' ~/ros2_ws/src/py_topic/setup.py > changed.txt && mv changed.txt ~/ros2_ws/src/py_topic/setup.py
cd ~/ros2_ws || exit
pip3 install zmq
colcon build
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

#Install micro-ros
cd ~
mkdir micro_ros_arduino
cd ~/micro_ros_arduino
source /opt/ros/$ROS_DISTRO/setup.bash
git clone -b main https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-path src --ignore-src -y
cd ~/micro_ros_arduino
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
