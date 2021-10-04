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
![image](https://user-images.githubusercontent.com/65916520/135898503-6236157b-193b-4d05-9e01-d1f49c156375.png)



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

## Add the RGB camera plugin
When you add the camera inside the infrared sensor, you will see that you have a lot of different types and settings to modify the data. The format we are using is `L_INT8`.

There is a lot of format such as:
```
UNKNOWN_PIXEL_FORMAT 	
L_INT8 	
L_INT16 	
RGB_INT8 	
RGBA_INT8 	
BGRA_INT8 	
RGB_INT16 	
RGB_INT32 	
BGR_INT8 	
BGR_INT16 	
BGR_INT32 	
R_FLOAT16 	
RGB_FLOAT16 	
R_FLOAT32 	
RGB_FLOAT32 	
BAYER_RGGB8 	
BAYER_RGGR8 	
BAYER_GBRG8 	
BAYER_GRBG8 	
PIXEL_FORMAT_COUNT 
```
You can find one of those to meet your needs in the `<format>`. As the setting inside the camera, it focus on 1x1 which will generates the data output of RGB value.

```
      <sensor name="rgbd_camera" type="rgbd_camera">
          <update_rate>5</update_rate>
          <topic>IR1</topic>
          <camera>
            <horizontal_fov>1.2</horizontal_fov>
            <image>
              <width>1</width>
              <height>1</height>
              <format>L_INT8</format>
            </image>
            <clip>
              <near>0.02</near>
              <far>10.0</far>
            </clip>
          </camera>
      </sensor>
```

Using the ROS2 output:
```
header:
  stamp:
    sec: 1398
    nanosec: 177000000
  frame_id: freenove_smart_car/right_IR/rgbd_camera
height: 1
width: 1
encoding: rgb8
is_bigendian: 0
step: 3
data:
- 36
- 48
- 27
---
```

The data is in an array output. 

## Change the pose/move the model to anywhere.

Let's use our [model.](https://github.com/feagi/feagi-core/blob/feature-controller-cleanup/third_party/gazebo/smart_car/freenove_4wd_car_description/models/sdf/freenove_smart_car.sdf#L221)

The line 221 allows you to move anywhere. It's highly encouraged to use X, Y only.

Using the picture below, the X and Y coordiation is this:

![image(16)(3)](https://user-images.githubusercontent.com/65916520/135904575-e05df68d-319f-4b47-905e-7823d988261b.png)

The command is called `<pose>` where it allows you to put it wherever you want
So,
<pose> X, Y, Z, R, P, Y </pose>
 
 
# Create from scratch on your own robot. 

## Create a world in Citadel
This is usually the best way to start the world so you can create a model inside the <world>. 

Here is the empty template:
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="template">

  </world>
</sdf>
```

SDF file should always have the start line with `<?xml version="1.0" ?>` so the program can know that you are using the xml format. After that, you should also have this line under the first line, `<sdf version="1.6"></sdf>`


Finally, you can add the world like this:
`<world name="template"></world>`

You can always change the name "template" to any name you want. Once you complete that, you can add some parameters of physics, sun and GUI. You can take a look at our current SDF file.
[More information about some detailed physics and GUI](https://ignitionrobotics.org/docs/citadel/sdf_worlds)

To start with the floor, you will need to create a model. I'm using the snippet code from our current file:
```
    <model name="ground_plane">
      <static>true</static>

    <link name="ground">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
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
        </visual>
      </link>
      </model>
```

This is the code snippet from our file.

Here is the empty template for you to use:
```
    <model name="ground_plane">
        <static>true</static>
        <link name="link">
        </link>
    </model>
```

Be sure this has to be inside the `<world></world>`
like this:
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="template">

    <model name="ground_plane">
        <static>true</static>
        <link name="link">
            
        </link>
    </model>
      
      
  </world>
</sdf>
```

As you can see that I put TRUE inside the `<static>`,
So that way, it won't move at all. 

So the model will need to have two things: <visual> and <collision>. Model needs this to illustrate the object. <material> (visual only), <geometry> and <pose>.
<Material> is pretty much sum up in the "Upload image in Ignition Citadel" sub-section.

There is a [lot of stuff](http://sdformat.org/spec?ver=1.8&elem=geometry) can be made within <geometry>. For the floor, we use <plane> like this:
```
<visual name="visual">
  <geometry>
    <plane>
      <normal>0 0 1</normal>
      <size>10 10</size>
    </plane>
  </geometry>
```

The line is called `<pose>` where it allows you to put it wherever you want
So,
<pose> X, Y, Z, R, P, Y </pose>


## Create a robot inside the <world>

You can have more than one model inside the <world>. The freenove_smart_car.sdf using 2 models, a robot and a track.

The robot will need joints. Each robot has various number of joints. A wheel, servo, arm, or motor. They all are the same thing to Gazebo's perspective. 

Freenove_smartcar has 6 joints total, 4 wheels and two servos.

Here is the example with the Two wheels on the floor:
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="template">

    <model name="ground_plane">
        <static>true</static>
        <link name="link">

        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        </link>
    </model>

    <model name="my_custom_robot">
        <link name="wheel_1">
        <pose>0 0 0 0 0 0 </pose>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.0325</radius>
              <length>.02</length>
            </cylinder>
          </geometry>
        </visual>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>0.0325</radius>
              <length>.02</length>
            </cylinder>
          </geometry>
        </collision>
        </link>

        <link name="wheel_2">
        <pose>0 0.2 0 0 0 0 </pose>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.0325</radius>
              <length>.02</length>
            </cylinder>
          </geometry>
        </visual>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>0.0325</radius>
              <length>.02</length>
            </cylinder>
          </geometry>
        </collision>
        </link>

      <joint name='two_wheel_together' type='revolute'>
        <parent>wheel_1</parent>
        <child>wheel_2</child>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>

    </model>
  </world>
</sdf>
```

As you can see this created two objects on the floor. If you get one of the wheel move, other wheel will be also moving due to the `<joint>`


# Set up with the ignition and ROS2 FOXY on your robot
There is several things you need to set up with ROS/IGN with your own robot:
1) Launch file
2) CMakeLists.txt
3) Package.xml
4) SDF file
5) setup.py
6) setup.py

## Launch file:
Here is the empty template using IGN/Foxy:
```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_ign_gazebo_demos = get_package_share_directory('[YOUR_CUSTOMIZE_FILE]')
    pkg_ros_ign_gazebo = get_package_share_directory('[YOUR_CUSTOMIZE_FILE]')

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r [YOUR_CUSTOMIZE_FILE].sdf'
        }.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,

        bridge,
    ])

```

Replace the `[YOUR_CUSTOMIZE_FILE]` include brackets with your directory. Replace the `[YOUR_CUSTOMIZE_FILE].sdf` to your current sdf file. 

`bridge` is the conversion which is already mentioned in "Connects with ROS2/Ignition" sub-section. 
You can add your topic @ ros2 @ ign

You can add as many as you want. There is no limit on topics so IGN and ROS2 can handle as many topics as you need. 

The current file we have are 11 topics being used at once. 




