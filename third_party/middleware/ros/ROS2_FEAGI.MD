# Ros2 on FEAGI
This is to test the sample on FEAGI. You will learn that FEAGI will react to sonar sensor. Once sonar sensor is stopped, you can see FEAGI stopped as well too. The purpose of this document is to learn how to use FEAGI through ROS. This document can be done using Micro-ROS or py2arduino.

## Raspberry PI + ROS2 on FEAGI
If you haven't set up ROS on Raspberry PI, you can run `./Raspberry_ROS_setup.sh` in your terminal. It will download Foxy bare in your raspberry pi along with the workspace to work with FEAGI and sensors. This method is almost exact same as local.

### Py2arduino on RPI

To start with the Raspberry PI on FEAGI:
1. Have FEAGI running already.
2. Open a new terminal, plug the board in and run those lines:
```
source ~/ros_ws/install/setup.bash
ros2 run py_topic sonar_sensor
```
3. Open a new terminal again and run those lines:
```
source ~/ros_ws/install/setup.bash
ros2 run py_topic py2arduino
```
4. Verify if FEAGI reacting to it. Once you verify it, it's working.

### Micro-ros on RPI

Work in proces

## Test with container

Work in process

## Work with local only

## Test with FEAGI
1. Follow the document from [this](https://github.com/feagi/feagi/blob/develop/DEPLOY.md)
2. Once you have FEAGI running, open the new terminal and follow one of two sections below:

## Py2ardiuno on FEAGI
1. Have FEAGI running already.
2. Open a new terminal, plug the board in and run those lines:
```
source ~/ros_ws/install/setup.bash
ros2 run py_topic sonar_sensor
```
3. Open a new terminal again and run those lines:
```
source ~/ros_ws/install/setup.bash
ros2 run py_topic py2arduino
```
4. Verify if FEAGI reacting to it. Once you verify it, it's working.

**Information information:** This using the file called ardiunotopython.ino. This file has to be upload to the board from Arduino IDE. This **will** not work on Micro-ROS.

## Micro-ROS on FEAGI
1. Have FEAGI running already.
2. Open a new terminal then do those lines:
```
source ~/ros_ws/install/setup.bash
ros2 run py_topic micro-ros 
```
3. Open another terminal and run; source ~/micro_ros_arduino/install/setup.bash
4. Plug the usb to the board where you uploaded micro-ros sample from and run in the terminal; ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6
5. See FEAGI's output changes as the sonar sensor reads the distance.

**Important Information:** This using the file called micro-ros_publisher.ino. This file has to be upload to the board from Arduino IDE or CLI. This will not work on ardiunotopython. 
