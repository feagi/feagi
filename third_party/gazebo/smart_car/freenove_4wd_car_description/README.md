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

In Ignition gazebo perspective, each joint is being controlled by the plugin called [joint_controller](https://ignitionrobotics.org/api/gazebo/4.1/classignition_1_1gazebo_1_1systems_1_1JointController.html). 

## Foxy
This is the latest Ros2 and everything has been designed in Foxy. **Galactic hasn't been tested by this project.**
Everything in this project is mainly focused on Foxy and has been uses Foxy. The model created plugins from IGN Gazebo and Foxy will detect everything from the robot once it is loaded. 

The topics being created by the model in IGN Gazebo are the following:
```
/IR0/image
/IR1/image
/IR2/image
/M0
/M1
/M2
/M3
/S0
/S1
/ultrasonic0

```

Each will be used in Publisher and Subscriber. The robot is currently named _freenove_smart_car_. 

Subscriber and Publisher are in the controller.py.

Within the controller.py:
```
Subscribers:
/Ultrasonic_0
/Infrared_0
/Infrared_1
/Infrared_2

Publishers:
/M0
/M1
/M2
/M3
/S0
/S1
/ultrasonic0
```
Here is the full diagram of Foxy/Citadel:
![image](https://user-images.githubusercontent.com/65916520/135670895-86912742-d4b6-45d4-adb5-b8a6996193e0.png)


The parameter is actually double data type. Since there is no double data type in ROS2, so IGN, ROS2 and Python using Float64.

M0, M1, M2, and M3 are the single wheel. M0 M2 are the right side and M1 M3 are the left side.

The combo motors are backward, forward, left, and right. They move all the wheels at once.

The servo are S1 and S0

Ultrasonic read the data from the plugin inside the model. It can read up to 3.5 m.



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

# Extra information + customize model using FEAGI

## Connects with ROS2/Ignition
The file called `freenove_smart_car.launch.py` inside the `launch/`. It's the bridge where you allows ros2 and ign to communicate with each other.

This example is from #65 in the file;
`/M3@geometry_msgs/msg/Twist@ignition.msgs.Twist` 

`/M3` is created in `models/sdf/freenove_smart_car.sdf`

So, Ignition and ros2 needs to know what it is for. You add @ after `/M3` which is `/M3@std_msgs/msg/Float64@ignition.msgs.Double` to let ros2 know that it's used by ros2 with std__msgs-msg-Float64. You will need to convert ros2 to ignition which is also `/M3@std_msgs/msg/Float64@ignition.msgs.Double`

This allows you to convert and communicate between ros2 and ign. 

**Important information: Everytime you add the new topic, be sure to add this on the launch file**


## Connects with FEAGI using Ign/Ros2
Unlike Launch file, FEAGI comes from the `Router.py` where this is the middle between ROS2/IGN and FEAGI. `Controller.py` is the one handles ROS2 topics.

Here is the diagram:


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



## Create topics from SDF model
Under the "Plugin parts" comment  in sdf file, there's a multiple plugins where it says the variable after `<topic></topic>`. Not only that, it is also created by sensors as well. With those, you can create a topic after you add the plugin. All available plugins are [here on their doc](https://ignitionrobotics.org/api/gazebo/4.1/namespaceignition_1_1gazebo_1_1systems.html)

As for the sensor part, the current available sensors are [here on their github](https://github.com/ignitionrobotics/ros_ign/tree/foxy/ros_ign_gazebo_demos)

There will be new sensors added to ignition one day. As Oct 1st 2021, this is the current resources we have.

In our case, freenove_smart_car are using Infrared, ultrasonic HC-SR04, motors, and servo. So, we have to use RGB camera to act like IR in ign, joint_controllers for servo and motors using their own mathematical formula convert in python, and lidar to act like ultrasonic.

In Servo, it using 90 degree only. The image illustrate what servo can do in degree.
![image(16)(1)](https://user-images.githubusercontent.com/65916520/135889076-fc3cb87c-6fef-4a96-a0f6-b46f58d4cb60.png)

Servo using the degree formula: `rad * 3.14159265359 / 180`

RGB is using 1x1 so the data will be simply [RGB](https://www.rapidtables.com/web/color/RGB_Color.html).

Motors are using joint_controller so it moves by a double data type value. 

Ultrasonic using the range so it's going to measure up to 4.5 meters. It can be seen in rviz as well.


## Upload image in Ignition Citadel
The floor is actually using the customize Neuraville's image. The wheels are also using the image as well too. 
The image has to be inside the `/model/sdf/` along with the [freenove_smart_car.sdf](https://github.com/feagi/feagi-core/tree/feature-controller-cleanup/third_party/gazebo/smart_car/freenove_4wd_car_description/models/sdf)

The file can be JPG or PNG. 

Unlike Ignition, Classic Gazebo using `<script></script>`. It's no longer supported by Iginiton so we use `<pbr>` now. This is how you upload the image on an object:
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
After the `<geometry></geometry>`, add this above. **Do not put this in the <collision> or your ignition will crash immediately**

## Add Joint_controller inside your sdf
You will need to add the joint_controller after the `</joint>`

In our sdf, it is set up as a joint_controller like this:
```
    <plugin
          filename="ignition-gazebo-joint-position-controller-system"
          name="ignition::gazebo::systems::JointPositionController">
          <joint_name>rear_right_wheel_joint</joint_name>
                <topic>M3</topic>
    </plugin>
```

Plugin is needed as you let ignition to know which plugin wil it use. Be sure that the `<joint_name>` is using the joint's name. `<topic>` is the topic where you can name anything you want for ROS2. 

## Add the 
