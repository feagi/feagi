# Neurorobotics with ROS2 and Gazebo Ignition
## Neurorobotics
Neurorobotics is a combination of artififical intelligence and robotics. It's using the neural systems to control the robot. The goal for neurorobotics is to be able to control the each connection it has. Not only that, it's train the artificial intelligence through the artifical spiking neural network.

What is the difference between automation robotics and neurorobotics? Robot term is a very broad subject. Robotics are often programmed by people and allowed others to control it. Automation will do things continiously without having anyone input. It's a basically closed loop. Neurorobotics do use the automation closed loop but it's can be also self-corrected which is something automation can't do.

In our case, it's FEAGI (Framework for Evolutionary Artificial General Intelligence) to make our robot as a neurorobotic. So with the physical robot or simulation robot, FEAGI needed some brain robot intertractions in a lower level where it needed some translation of the spike train from each inviduial neurons. So, it needs to control each component such as each mechanically and electrically hardware such as motors, and servos and each sensors. So, the translation for neuron comes from the FEAGI network where it can battery_translator to communicate with the motors or sensors. 

The system process on FEAGI is very straightforward yet can be very complicated. So, FEAGI is learning and has a memory so once you trained FEAGI, FEAGI can control what it learned

So, to make robot work with the FEAGI, we need to have a multiple integration between hardware and software. Artificial intelligence is a very high level where it needed both to intergrate and able to learn things. An automation and programmed are something it can move the robot. So, we need to have a software where it can communicate with the hardware and software as a middleware. FEAGI is a software built by python3 enitrely. 

ROS2 is also heavily supported Python3 which is in FEAGI's advantage. ROS2 is excellent at control and communicate with a muliple hardware that can be done up to 100 or even more. This is the essential part for FEAGI. FEAGI and ROs2 can be integrated pretty simple.

As for the hardware side, this raise a lot of complicated questions and answers. Even if they can be low level to high level, the integration is a lot of multiple conversions. ROS2 is very well with the conversions as well. When the hardware is mentioned, it doesn't mean the physical hardware, but it is also include the simulation robot. Those can generate the data where FEAGI and ROS2 needed. So, we designed the model to mimic as close as possible of the freenove smart car include their parts. 

I will discuss about ROS2 first. There are multiple of reasons why we started to use ROS2. The number one reason is because ROS2 provides real-time data where FEAGI heavily relies on the real-time data.

ROS2 is compatible with industrial safety applications. Not only that, ROS1 is approaching its end of life which is in 2025. ROS2 is the newest version and FEAGI is purely made by Python3. This is important to know because ROS2 also relies on Python3. ROS2 has a unique feature called QoS where you can reduce or increase the data output. ROS1 doesn't have it. These are the reasons why we went this route.

As for the Igition Citadel, we picked Citadel due to the wealth of support from the Gazebo team. Second, Gazebo 11 is also approaching its end of life which is also in 2025. Third, in general, Ignition Citadel is designed for ROS2 as well. Finally, Ignition isn't the same as the classic version of Gazebo11 therefore, to start with Ignition would allow for the smoothest possible transition.

 In summary, ROS2 FOXY, and Ignition Citadel are very good for our resources and fit FEAGI's needs.

The conversion between Ignition and ROS2 is very straightforward and relatively simple, which is a huge advantage for us. There are multiple conversions to integrate the programs. This is a perfect plan for any company who would want and need to make their robots compatible with FEAGI in the future.

I would like to explain in more detail each section from the README.md.

## An Explanation For The Link VS A Joint Section
There is one thing to remember. It is imperative to the close parameters like `<link></link>` or you will recieve an error and be unable to start the ign gazebo. Also, you must remember that the link is just a part. It wouldn't impact anything so if you want to use something to move the link, use the `<joint></joint>` This allows you to control the joint using any plugins.

If you want to create a topic, <joint> and <link> can't be created as a topic. You may create a topic name from the plugin or the sensor. 

As for the wheel, it is usually best to attach the two links together. In our robot, we used the chassis as our parent link and the wheel as our child link inside the `<joint>`.

 Using an actual example from our robot’s M2 (Rear_left_wheel):
``` 
   <joint name='rear_left_wheel_joint' type='revolute'>
        <parent>chassis</parent>
        <child>rear_left_wheel</child>
        <axis>
          <xyz>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower> <!--negative infinity-->
            <upper>1.79769e+308</upper> <!--positive infinity-->
        </limit>
        <dynamics>
          <damping>0.01</damping>
          <friction>1</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
        </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
          <cfm_damping>1</cfm_damping>
        </ode>
      </physics>
      </joint>
```

 As you can see, there are some settings for the physics inside the <joint></joint>. 
 
 
 Let's focus on a few lines.
`<axis>` is the parameter where you can move the robot. In our case, we used this to move the y where it allows the motor to continue operating like a motor. 
`<limit>` is the limit where you define to keep the position pointer. Originally, I left this blank because it should be working but then I realized that this was causing some issues with the position pointer and joint_controller so I added this to keep it counting properly without making the simulation robot bounce.
`<damping>` is the line where you can reduce the jumpiness. 
`<implicit_spring_damper> and <cfm_damping>` if set to True (or 1), it will enable gazebo to simulate the stiffness and damping.
[More information here](http://sdformat.org/spec?ver=1.8&elem=joint#ode_implicit_spring_damper)


## An Explanation For The Image:
This took me a lot of time to accomplish. Eventually, I realized that the `<script>` isn't supported on Ignition anymore. I've spent a lot of time on Gazebo 11. As previously stated, Gazebo 11 and Ignition Citadel aren't the same. There several different syntaxes and I used the new syntaxes on Ignition. Unfortunately, when you have used the `<script>` on the Ignition, it does not display any error. It simply take the inputs and runs with the invisible error. As you may now be aware, Iginition is still new so feel free to provide feedback and inform their team of any new bugs you find. 🙂 

 I found out that it's using <pbr> only. Here is the code of the actual robot in our file:
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

 Furthermore, I would like to expand more on the information about the ambient, diffuse, specular, pbr, and metal. 

`ambient diffuse specular` has to do with the lighting or as we, normal people, refer to it as the reflection. Ambient has some light inside it and will diffuse to allow a bright surface. Specular is glossy which cause some level of reflection. You can delete one of them or keep them all, depending on your particular situation. 

 `pbr` is the line where you can import any image. Take a look at our robot's wheel and the floor. Those are being imported by a picture. We designed this picture as well.
 
 There are two under `<pbr>` called metal and specular. As of right now, specular is a bug which is why I am using metal to import the picture.

## An Explanation For Sensors:
There are a lot of things to cover about joint_controller and sensors. Unfortunately, since the Ignition is still new, there are currently very limited resources on this topic. As a result, we have been required to think outside the box. 

The ultrasonic in our robot is actually using Lidar. There is no official ultrasonic plugin. In order to accomplish this, I had to replace the few settings inside the ultrasonic sdf and make the range to 4 m distance in order to keep the small cone and use the ROS2 feature called sensor_msgs/msg/Range. Our file is using the float32 range. This allows us to read the distance in real time. 

The infrared sensors are actually using an RGB camera. This case is a very unique case since IR is very rare and almost non-existence so I came up with a way to use the RGB camera and convert it to focus on the one pixel which is 1x1. This allows RGB camera to pretend like it is an IR in gazebo! WIth the 1x1, it gives the RGB data like (R,G,B). 

The concept is actually simple using an RGB camera in the gazebo by converting them into a true/false logic based on the intensity of the values recieved from the ROS2 data which is using the sensor_msgs/msg/Image in ROS2. 

Here is the full description:
```
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be the optical frame of camera
                             # origin of frame should be the optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

```

## Create topics within SDF for ROS2
This might be tricky to explain, but I wanted to discuss this anyway. Essentially, you can have the same topic name from ROS2 and SDF. In our case, we used Python inside ROS2 to subscribe the data to SDF while the SDF topic creates the value and delivers the data to ROS2.  This can be done inside the launch file. The conversion between ROS2 and IGN played a big role in this part. Without it, you will unable to see the data from Ign on ROS2.

ROS2 and Ign will simply convert the data from SDF topic to ROS2 topic without overwriting anything; therefore, linking the name to together wouldn't propose any issue since they need to be match. IGN topic is considered as a publisher so we can just subscribe the topic. 

## Diff Driver VS Joint Controller On Wheels
There is a lot of important aspects between both plugins. diff driver is a plugin where it allows you to control the velocity using x linear and z angular. The difference between diff and joint controller is that diff moves two wheels while the joint using one wheel to move. You can't use 1 or 4 on diff driver since it needs 2 wheels. When you run one command, both wheels will move. I was trying to control one wheel only but that was unlikely. Thus, it required me to use the workaround by inserting the double wheels (actual wheel and invisible wheel) where you can just put the empty joint inside the invisible wheel without having it effect the visual and collision. 

For example:
```
Actual wheels    Invisible wheels
M0 M1         | M0_inv M1_inv
M2 M3         | M2_inv M3_inv

You connect 4 diff driver plugins below using diff_drive.
M0 - M1_inv
M1 - M0_inv
M2 - M3_inv
M3 - M2_inv
```

 This design allows you to control one joint and trick the diff driver plugin. The negative thing about this is that it requires a lot of space in the code compare to the joint_controller where it just needs one plugin for a wheel which is 4 plugins in total compared to 4 plugins with the diff driver plus 8 joints.
 
Not only that, we need to be more careful with the speed on diff driver because it could make robot jumpy and bounce unlike joint controller plugins.

We need also put zero on wheels' input everytime we are done with it or it will continue until the program closes or we put zero in the input. Joint controller doesn't need us to put in a zero once we are done with it. It can stop once the input has been met which gives us a large advantage.

Diff driver requires us a lot of workaround. Joint controller did not require a workaround and the best method to use on wheels.

Therefore, this is why you see our robot being used with the joint controller. 

## Why Teach Others About ROS2 and IGN
When I started with ROS2 and Gazebo11, there was very little documentation and it was very difficult to intergrate between ROS2 and the classic Gazebo. I spent almost a full month trying to figure out how to use Gazebo11 and Foxy and I hadn’t reached halfway. I eventually learned that Gazebo11 is the classic Gazebo, and the Gazebo uses Ignition Gazebo (which is the future for the robot industrials in my opinion). There are more examples being done between Foxy and Ignition Citadel so that's how I was able to learn and intergrate them. Classic Gazebo is actually designed for ROS1 only. I mean, you can use it on ROS2, but it requires a lot of data to convert to integrate and there is very little documentation about that one. Foxy and Citadel are also very new, so there is little documentation for these bridge between them as well.

As a result, I was thinking, “why don't we just create something and help others to learn how to intergrate their own projects”? I figured that if we create this documentation and help the Gazebo/ROS2 community out then we can have more tutorials and helpful documentation for future reference.  

I'm a huge advocate on using the newest and latest software where we can continue to create using our own creativity. I like how ROS2 and Ignition are very flexible with all the changes, especially how it integrates with Arduino, Python3, and Raspberry pi (which are very popular hardware).

## FEAGI

FEAGI is actually designed to adapt with any hardware including Raspberry Pi, Arduino, and Gazebo. FEAGI is an artificial intelligence where it can be used on any robot and hardware. ROS2 plays a big role where it allows FEAGI to improve themselves using the hardware as their body. They are created to be affordable for companies or personal households. This can be used on any type of machine as long as it can feed the data to FEAGI. 

 If you want to test it on your Arduino or Raspberry Pi, it's located in `feagi/third_party/arduino/`

 ## FEAGI troubleshooting tools
 
 There is frequently situation where you happen to have a bug, problem or some errors somewhere. Let's use two examples
 
 First example:
 Here is the actual design between FEAGI and robot in gazebo.
 
 ![Etsy Item Listing Photo(3)](https://user-images.githubusercontent.com/65916520/145626055-472fb83e-41b6-4f18-bb27-56ce1d188d92.png)
 
 So when you are having some issue with the robot or it's not doing what you were expected to see. 
 
 So, you would start to troubleshooting and find what is wrong like this:
![Etsy Item Listing Photo(4)](https://user-images.githubusercontent.com/65916520/145626243-9fde2409-6105-4212-8246-0f9d9929c063.png)

Without the FEAGI tools, you have three ways to troubleshooting in this case:
 1. Check model in gazebo's file or use the viewer monitor.
 2. Load topic list in ros2 and run some echo each sensor.
 3. Debug the python3 code or print each data inside the python3 to see the data. 
 
 This list is not very good approach if you have more than 10 sensors or even 30 sensors. Imagine you had to open a new terminal, type `ros2 topic echo <sensor>` then re-do the step over and over like this
 ```
 ros2 topic echo /M0
 ros2 topic echo /M1
 ros2 topic echo /M2
 ros2 topic echo /M3
 ros2 topic echo /M4
 ros2 topic echo /M5
 ros2 topic echo /M6
 ros2 topic echo /ultrasonic
 ros2 topic echo /microphone
 ros2 topic echo /battery
 ```
 and possibly more just to find one bug. This will waste a ton of time and energy to chase a bug through many parts. If you decided to create a script to load all at once, it wouldn't be scalable due to new parts in future or upgrade the robot. You will have to add more as sensors are being added. What if the name of sensor is changed? What if the ros2 or gazebo was not the issue and it was the python code? What if it was not the python code and it was Gazebo issue? Too many ways to chase one error. 
 
 With the FEAGI tools, it is scalable and able to load all sensors at once. 
 
 ![Etsy Item Listing Photo(5)](https://user-images.githubusercontent.com/65916520/145627083-70c1bade-4825-4402-a952-f2acfb597f45.png)

FEAGI tools simply don't interrupt the data nor change anythihng between FEAGI and project. 
So, it will load Grafana and Godot to see everything at once. With the grafana, you will see the data being deliver. You will see which sensor is being used in godot instantly. If you add new sensor, it will be added automatically. So, let's say..if you see that motor is being used in godot while you don't see FEAGI using it, you would know it was python due to error. So, if you see the ultrasonic is not getting anything data from FEAGI, you would know it's gazebo issue due to misposition in the gazebo model. This tool is designed to help you to troubleshootiong everything at once. This is scalable with any new robot as long as you defined them. 
 
 Another example, let's imagine that your robot is using motors, lidar, and some more sensors. So, the robot will be walking around the room and then suddenly, it stopped. There was no object in the way. You would probably assume it was code. With this FEAGI tool, you can just see what sensor is being used and you will see that lidar is seeing something. You aren't seeing any object in the robot's way. So, you would assume it's hardware. The first thing you would do is to wipe the lidar and you will see that the cortical area of lidar in feagi tool disappeared. It helped you to find the issue instantly. You would know that the code wasn't the cultprit. Some smudge on lidar was the cultprit. You saved a lot of time. 
 
 More information of feagi tools which is located in [feagi-insight](https://github.com/feagi/feagi-insights/tree/develop)
