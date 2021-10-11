# Building A Robot Simulation Using ROS2 and Gazebo Igition
## Requirements For This Project:
1. ROS2 Foxy
2. Citadel for Ign Gazebo
3. Python3
4. Linux (Currently supported on Linux. Windows/Macs will eventually be added)

## Starting With A Car In Simulation
There are two important aspects of an SDF file. One is the world and the other is the model. The world is more likely the enviroment you created for the robot. A model is the object (in our case, a robotic car)
To learn more about the world and a model, here are some Ignition tutorials available:
[World](https://ignitionrobotics.org/docs/citadel/sdf_worlds)
[Model](https://ignitionrobotics.org/docs/citadel/building_robot)

## A Link VS A Joint
A `<link>` is the part where you create a new part. Our model has four wheels, front_left, front_right, rear_left, and rear_right. So, with the <joint>, this allows you to put two parts (or link) together. I added all the wheels to attach on the body (chassis).

## Instructions On Adding The Image:
Igniton Citadel is using `<pbr></pbr>` to allow you upload the image. From our current file:
```
<material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>1 0.8 0.8 1</specular>
            <pbr>
                <metal>
                  <albedo_map>floor.png</albedo_map>
                  <normal_map>floor.png</normal_map>
                </metal>
              </pbr>
          </material>
```

The picture is inside the `models/sdf/` folder since that is where the sdf file is at. It follows the path where the sdf is in. 

## Measurement Inside The Citadel
The default measurement is meter. Everything is done in meters so be sure to use the appropriate measurement to match your real life custom robot. 

# FEAGI On A Car Model
## Instructions On Starting With This Project:

1. navigate to __freenove_4wd_car_description/__ directory
2. `source /opt/ros/foxy/setup.bash`
3. `colcon build`
4. `source install/setup.bash`
5. `ros2 launch freenove_4wd_car_description freenove_smart_car.launch.py`

If you want to move it using the python, do it on a different terminal
1. Navigate to __freenove_4wd_car_description/__ again
2. `source install/setup.bash`
3. `ros2 run freenove_4wd_car_description controller.py`

![demo](https://user-images.githubusercontent.com/65916520/136594848-ce55fcf3-7f2c-422b-9253-a61b095956ef.gif)

## Instructions on starting with this project in the container

1. Navigate to __/feagi-core/docker/__
2. `docker-compose -f docker-compose-feagi-ros-ign.yml build --no-cache`
3. Until #2 is complete, run this `docker-compose -f docker-compose-feagi-ros-ign.yml up`
4. [Open this link](127.0.0.1:6080/) to connect from your container with your browser
5. Open terminal inside the container then type this `./setup_simulation.sh`


## Information on Models:
This model was designed to look as closely as the real life robot named [Freenove 4wd smart car](https://www.amazon.com/Freenove-Raspberry-Tracking-Avoidance-Ultrasonic/dp/B07YD2LT9D).
The purpose for this was to allow the robot to understand the enviroment it is in and illustrate the situation in a rviz/gazebo. This is using several integrations between the Igniton Gazebo Version Citadel, Ros2 Foxy and FEAGI. This robot has 3 infrared sensors, Ultrasonic (HC-SR04), two servos, and four wheels. This model is controllable too.

![image](https://user-images.githubusercontent.com/65916520/136081103-f7f106e7-5e22-4b15-b3e9-cbfbc75b89fd.png)

## Instructions On Adding More Than Two Wheels:
You can have as many wheels as you want. You can even start with the one wheel too. This section will teach you how to add more wheels.

So our model has four wheels, front_left, front_right, rear_left, and rear_right. Those are /M0, /M1, /M2, and /M3 respectively to ROS2 perspective. 


## To Add Sensors:
There are several sensors embedded on the robot: Infrared and ultrasonic sensors. 

**Ultrasonic**: the simulation is designed to mimic ultrasonic as closely as possible. The range is between 2 cm to 400 cm which is 0.02 m to 4.5 m. It is displayed in RVIZ as well. The red line should look like this:
(RVIZ picture insert)

**Infrared sensor**: This has three infrared sensors inserted on the purple plate/board. Left, middle, and right which called `/IR0, /IR1, and /IR2` respectively. It's configured as infrared sensors.

## Understanding Plugins:
Plugins are from Ign Gazebo's library. Here is the current resource for [Foxy and Citadel plugins](https://github.com/ignitionrobotics/ros_ign/tree/foxy/ros_ign_gazebo_demos)


## Joint_controller:
The joint controller is a plugin where it allows you to control the arms, joint, or any single joint. It was designed to focus on the single joint to be controlled by ROS2. The current acceptable input is a float value.



# Extra and Detailed Information:
## Information About The FEAGI and ROS2/IGN
ROS2/IGN communicate with FEAGI agent which is `Controller.py` and `Router.py`. The conversion between IGN and ROS2 is in the launch file.
Here is the diagram between them:
![image](https://user-images.githubusercontent.com/65916520/136081295-5c9e09d0-7a95-4743-9294-c975038d8924.png)


## Ignition Gazebo
The version of Ignition A.K.A IGN is Citadel. Citadel is highly recommended and has been largely supported by the IGN Gazebo team. It is not the same as the classic Gazebo (Version 11 or less). IGN is designed to work with the Ros2 Foxy and newer versions. **This will not include ROS1 nor Python2.7**

This will be using an SDF file which is located in the __models/__ directory. The model has multiple plugins included in each wheel, servo, and ultrasonic. The image will display in the Foxy section of all plugins that is being sent to Foxy.

In Ignition gazebo perspective, each joint is being controlled by the plugin called [joint_controller](https://ignitionrobotics.org/api/gazebo/4.1/classignition_1_1gazebo_1_1systems_1_1JointController.html).

## Foxy
This is the latest ROS2 and everything has been designed in Foxy. **Galactic hasn't been tested by this project.**
Everything in this project is mainly focused on Foxy and has been using Foxy. The model created plugins from IGN Gazebo and Foxy will detect everything from the robot once it is loaded.


__TODO: Add problem/solution section, ROS2 stuff to make it work, topics explained in more details__
