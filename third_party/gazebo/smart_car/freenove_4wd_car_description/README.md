# FEAGI on a car model
## Instruction to start with this project:

1. navigate to __freenove_4wd_car_description/__ directory
2. source /opt/ros/foxy/setup.bash
3. colcon build
4. source install/setup.bash
5. ros2 launch freenove_4wd_car_description freenove_smart_car.launch.py

If you want to move it using the python, do it on a different terminal
1. Navigate to __freenove_4wd_car_description/__ again
2. source install/setup.bash
3. ros2 run freenove_4wd_car_description controller.py

(Include gif)

## A model information:
This model was designed to look as closely as the real life robot named [Freenove 4wd smart car](https://www.amazon.com/Freenove-Raspberry-Tracking-Avoidance-Ultrasonic/dp/B07YD2LT9D).
The purpose for this was to allow the robot to understand the enviroment it is in and illustrate the situation in a rviz/gazebo.  This is using several inttegrations between Igniton Gazebo Version Citadel, Ros2 Foxy and FEAGI. This robot has 3 infrared sensors, Ultrasonic (HC-SR04), two servos, and four wheels. This model is controllable too.

![image](https://user-images.githubusercontent.com/65916520/136081103-f7f106e7-5e22-4b15-b3e9-cbfbc75b89fd.png)


# How to build the simulation car
## Requirements for this project:
1. Ros2 Foxy
2. Citadel for ign gazebo
3. Python3 
4. Linux (Currently supported on Linux. Windows/Macs will eventually be added)

## To start with a car in simulation
There is two important aspect of SDF file. One is the world and the model. The world is more likely the enviroment you created for the robot. A model is the object (in our case, a robotic car)
To learn more about the world and a model, they are available on the ignition tutorials:
[World](https://ignitionrobotics.org/docs/citadel/sdf_worlds)
[Model](https://ignitionrobotics.org/docs/citadel/building_robot)

## A link vs A joint
A `<link>` is the part where you create new. Our model has four wheels, front_left, front_right, rear_left, and rear_right. So, with the <joint>, this allows you to put two parts (or link) together. I added all wheels to attach on the body (chassis). 



## Add more than two wheels
You can have as many wheels as you want. You can even start with the one wheel too. This section will teach you how to add more wheels. 

So our model has four wheels, front_left, front_right, rear_left, and rear_right. Those are M0, M1, M2, and M3 respectively to ROS2 perspective. We will go there shortly.








*TODO: add joint_controller, understand plugin, add an image, launch file, history with ultrasonic/IR sensors*


# Extra and detailed information:
## Information about the FEAGI and ROS2/IGN
ROS2/IGN communicate with FEAGI agent which is `Controller.py` and `Router.py`. The conversion between IGN and ROS2 is in the launch file. 
Here is the diagram between them:
![image](https://user-images.githubusercontent.com/65916520/136081295-5c9e09d0-7a95-4743-9294-c975038d8924.png)


## Ignition Gazebo
The version of Ignition A.K.A IGN is Citadel. Citadel is highly recommended and has been largely supported by the IGN Gazebo team. It is not the same as the classic Gazebo (Version 11 or less). IGN is designed to work with the Ros2 Foxy and newer versions. **This will not include ROS1 nor Python2.7**

This will be using an SDF file which is located in the __models/__ directory. The model has multiple plugins included in each wheel, servo, and ultrasonic. The image will display in the Foxy section of all plugins that is being sent to Foxy.

In Ignition gazebo perspective, each joint is being controlled by the plugin called [joint_controller](https://ignitionrobotics.org/api/gazebo/4.1/classignition_1_1gazebo_1_1systems_1_1JointController.html). 

## Foxy
This is the latest Ros2 and everything has been designed in Foxy. **Galactic hasn't been tested by this project.**
Everything in this project is mainly focused on Foxy and has been uses Foxy. The model created plugins from IGN Gazebo and Foxy will detect everything from the robot once it is loaded. 

