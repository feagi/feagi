# Micro-ROS set up
## Setup on linux on Teensy and Arduino
1. Plug the micro USB to Teensy 4.1 (the board should have an orange blinking)
2. Press the button on the board.(The blinking should stop)
3. Open terminal and follow this list:
```
wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
wget https://www.pjrc.com/teensy/td_153/TeensyduinoInstall.linux64
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
tar -xf arduino-1.8.13-linux64.tar.xz
chmod 755 TeensyduinoInstall.linux64
./TeensyduinoInstall.linux64 --dir=arduino-1.8.13
cd arduino-1.8.13/hardware/teensy/avr/cores/teensy4
make
cd ~/arduino-1.8.13/
sudo ./install
```
4. Open and test with Teensy.
5. If it runs without any issue, you are clear to start the section, "Add Micro-ROS to Arduino".


## Add Micro-ROS to Ardiuno
1. Download latest zip from (Micro-ROS)[https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v1.0.0]
2. Open Arduino IDE
3. Click Sketch > Include library > Add Zip > [add the zip you downloaded] 
4. 
```
Teensy- cd [Ardiuno PATH]/hardware/teensy/avr/
Ardiuno ZERO/DUE- cd ~/.arduino15/packages/arduino/hardware/sam/1.6.11 

```
5. curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
6. Open a file called "Micro-ros_publisher.ino" from example in Micro-ros library 
7. Paste this part under the comment says // create node
```
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 30;
  RCCHECK(rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops));
  //RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
```
8. Click Download in Arduino IDE 
9. Wait until it uploaded to the board then unplug the board.
10. Open two terminals then run in one of terminals: mkdir micro_ros_arduino 
11. cd micro_ros_arduino && git clone git@github.com:micro-ROS/micro_ros_arduino.git
12. colcon build
13. source install/setup.bash
14. Plug the board in
15. In #1 terminal: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6
16. in #2 terminal, source micro_ros_arduino/install/setup.bash then ros2 topic list
17. If you are able to see the node appears, it means you are able to communicating between arduino and ROS2.




# Problem section:
## Problem 1: 
WARNING: library micro_ros_arduino-1.0.0 claims to run on OpenCR, Teensyduino, samd, sam, mbed architecture(s) and may be incompatible with your current board which runs on avr architecture(s).

## Solution 1: 
Go to arduino-1.8.13/hardware/teensy/avr
Then paste this, 
curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt


# More information:
1. https://www.youtube.com/watch?v=ze-HiCr5s60&ab_channel=pk
2. https://www.pjrc.com/teensy/td_download.html
3. https://github.com/micro-ROS/micro_ros_arduino
4. https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html

