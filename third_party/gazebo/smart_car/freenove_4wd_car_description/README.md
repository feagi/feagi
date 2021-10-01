# Ign gazebo + Foxy + Feagi
## About this project
This model was designed to look as closely as the real life robot named [Freenove 4wd smart car](https://www.amazon.com/Freenove-Raspberry-Tracking-Avoidance-Ultrasonic/dp/B07YD2LT9D).
The purpose for this was to allow the robot to understand the enviroment it is in and illustrate the situation in a rviz/gazebo.  This is using several inttegrations between Igniton Gazebo Version Citadel, Ros2 Foxy and FEAGI. Each section will have more detailed information on each topic discussed below.

## Requirements for this project:
```
1. Ros2 Foxy
2. Citadel for ign gazebo
3. Python3 
4. Linux (Currently supported on Linux. Windows/Macs will eventually be added)
```

## Ignition Gazebo
The version of Ignition A.K.A IGN is Citadel. Citadel is highly recommended and has been largely supported by the IGN Gazebo team. It is not the same as the classic Gazebo (Version 11 or less). IGN is designed to work with the Ros2 Foxy and newer versions. **This will not include ROS1 nor Python2.7**

This will be using an SDF file which is located in the __models/__ directory. The model has multiple plugins included in each wheel, servo, and ultrasonic. The image will display in the Foxy section of all plugins that is being sent to Foxy.

## Foxy
This is the latest Ros2 and everything has been designed in Foxy. **Galactic hasn't been tested by this project.**
Everything in this project is mainly focused on Foxy and has been uses Foxy. The model created plugins from IGN Gazebo and Foxy will detect everything from the robot once it is loaded. 

The topics being created by the model in IGN Gazebo are the following:
```
/M1
/M2
/M3
/M4
/cmd_vel
/lidar
/model/vehicle_green/cmd_vel
/model/vehicle_green/odometry
/servo
/servo1
/ultrasonic
```

Each will be used in Publisher and Subscriber. The robot is currently named vehicle_green. 

Subscriber list:
```
/ultrasonic
```

And the file being used as subscriber:
```
range.py
```

Publisher list:
```
/M1
/M2
/M3
/M4
/servo
/servo1
/cmd_vel
/lidar (Pointcloud)
```

And the file being used as publisher:
```
motor.py
```

Here is the full diagram of Foxy/Citadel:
![image](https://user-images.githubusercontent.com/65916520/132761385-42e3c8c0-43e9-4e9c-9a52-96142438b023.png)

Each function and all information related to Python will be in explained in the Python3 subsection.


## Python3
The motor.py will provide several functions.
All functions:
```
M1T(DOUBLE, DOUBLE)
M2T(DOUBLE, DOUBLE)
M3T(DOUBLE, DOUBLE)
M4T(DOUBLE, DOUBLE)
head_RIGHT_LEFT(DOUBLE, DOUBLE)
head_RIGHT_LEFT(DOUBLE, DOUBLE)
backward()
forward()
left()
right()
```

The parameter is actually (x,z). As the way it is at this moment, it is being forward if the value is negative double which means it will be backward if the value is positive.

M1, M2, M3, and M4 are the single wheel. M1 M3 are the right side and M2 M4 are the left side.

The combo motors are backward, forward, left, and right. They move all the wheels at once.

The servo are head_RIGHT_LEFT and head_RIGHT_LEFT. 

Motor.py is considered as a publisher at this point to ROS2 perspective. As it created multiple topics at once, it will display the ros2 topic as well and controls the gazebo. 

Range.py displays the ultrasonic data and it's subscriber in ros2 perspective as well. This will connect to the topic called_/ultrasonic_automatically and read the data from the plugin inside the model. It can read up to 3.5 m.



## Instruction to start with this project:

1. navigate to __freenove_4wd_car_description/__ directory
2. source /opt/ros/foxy/setup.bash
3. colcon build
4. source install/setup.bash
5. ros2 launch freenove_4wd_car_description diff_drive.launch.py

If you want to move it using the python, do it on a different terminal
1. Navigate to __freenove_4wd_car_description/__ again
2. source install/setup.bash
3. ros2 run freenove_4wd_car_description motor.py

# Extra information + customize model using FEAGI

## Connects with ROS2/Ignition
The file called `freenove_smart_car.launch.py` inside the `launch/`. It's the bridge where you allows ros2 and ign to communicate with each other.

This example is from #65 in the file;
`/M3@geometry_msgs/msg/Twist@ignition.msgs.Twist` 

`/M3` is created in `models/sdf/freenove_smart_car.sdf`

So, Ignition and ros2 needs to know what it is for. You add @ after `/M3` which is `/M3@geometry_msgs/msg/Twist` to let ros2 know that it's used by ros2 with geometry/msgs/msg.Twist. You will need to convert ros2 to ignition which is also `/M3@geometry_msgs/msg/Twist@ignition.msgs.Twist`

This allows you to convert and communicate between ros2 and ign. 


## Connects with FEAGI using Ign/Ros2
Unlike Launch file, FEAGI comes from the `controller.py`

You can insert this snippet code in your customize code:
```
address = 'tcp://' + router_settings['feagi_ip'] + ':' + router_settings['feagi_port']
feagi_state = find_feagi(address=address)

print("** **", feagi_state)
sockets = feagi_state['sockets']

print("--->> >> >> ", sockets)

# todo: to obtain this info directly from FEAGI as part of registration
ipu_channel_address = 'tcp://0.0.0.0:' + router_settings['ipu_port']
print("IPU_channel_address=", ipu_channel_address)
opu_channel_address = 'tcp://' + router_settings['feagi_ip'] + ':' + sockets['opu_port']

feagi_ipu_channel = Pub(address=ipu_channel_address)
feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)
```

before your code.

Be sure to include this function as well
```
def send_to_feagi(message):
    print("Sending message to FEAGI...")
    print("Original message:", message)

    feagi_ipu_channel.send(message)
```

Take a look at controller.py and see how it's done.

## Create topics from SDF model
Under the "Plugin parts" in sdf file, there's a multiple plugins where it says the variable after `<topic></topic>`. Not only that, it is also created by sensors as well. With those, you can create a topic. 

As for the sensor part, the current 

