# Neurorobotics with ROS2 and Gazebo Ignition
## An Explanation of Why We Started to Use ROS2 Foxy and Ignition Citadel

 I will discuss about ROS2 first. There are multiple of reasons why we started to use ROS2. The number one reason is because ROS2 provides real-time data where FEAGI heavily relies on the real-time data.

ROS2 is compatible with industrial safety applications. Not only that, ROS1 is approaching its end of life which is in 2025. ROS2 is the newest version and FEAGI is purely made by Python3. This is important to know because ROS2 also relies on Python3. ROS2 has a unique feature called QoS where you can reduce or increase the data output. ROS1 doesn't have it. These are the reasons why we went this route.

As for the Igition Citadel, we picked Citadel due to the wealth of support from the Gazebo team. Second, Gazebo 11 is also approaching its end of life which is also in 2025. Third, in general Igntion Citadel is designed for ROS2 as well. Finally, Igntion isn't the same as the classic version of Gazebo11 therefore, to start with Ignition would allow for the smoothest transition.

In summary, ROS2 FOXY, and Igntion Citadel are very good for our resources and fit FEAGI's needs.

The conversion between ignition and ROS2 is very straightforward and relatively simple, which is a huge advantage for us. There are multiple conversions to integrate the programs. This is perfect plan for any company who would want and need to make their robot compatible with FEAGI in the future.

I would like to explain more details on each section from the README.md.

## An Explanation For The Link VS A Joint Section
There is one thing to remember. It is imperative to the close parameters like `<link></link>` or you will recieve an error and be unable to start the ign gazebo. Also, you must remember that the link is just a part. It wouldn't impact anything so if you want to use something to move the link, use the `<joint></joint>` This allows you to control the joint using any plugins.

In addition, if you want to create a topic, <joint> and <link> can't be created as a topic. You may create a topic name from the plugin or the sensor. 

As for the wheel, it is usually best to attach the two links together. In our robot, we used the chassis as our parent link and the wheel as our child link inside the `<joint>`.

Using the actual example from our robot's M2 (Rear_left_wheel):
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

As you can see, there are some settings for the physics inside the wheel. 

Let's focus on a few lines.
`<axis>` is the parameter where you can move the robot. In our case, we used this to move the y where it allows the motor to continue operating like a motor. 
`<limit>` is the limit where you define to keep the position pointer. Originally, I left this blank because it should be working but then I realized that this was causing some issues with the position pointer and joint_controller so I added this to keep it counting properly without making it bounce.
`<damping>` is the line where you can reduce the jumpy. 
`<implicit_spring_damper> and <cfm_damping>` if set to True (or 1), it will enable gazebo to simulate the stiffness and damping.
[More information here](http://sdformat.org/spec?ver=1.8&elem=joint#ode_implicit_spring_damper)


## An Explanation For The Image:
This took me a lot of time to accomplish. Eventually, I realized that the `<script>` isn't supported on igntion anymore. I've spent a lot of time on gazebo 11. Like I said earlier, gazebo 11 and igntion Citadel aren't same. There several different syntaxes and I used the new syntaxes on igntion. Unfortunately, when you have used the `<script>` on the igntion, it does not display any error. It simply take the inputs and runs with the invisible error. As you may now be aware, Iginition is still new so feel free to provide feedback and inform them of any new bugs you find. 🙂 

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

I would like to expand more on the information about the ambient, diffuse, specular, pbr, and metal. 

`ambient diffuse specular` has to do with the lighting or as we, normal people, refer to it as the reflection. Ambient has some light inside it and will diffuse to allow a bright surface. Specular is glossy which cause some level of reflection. You can delete one of them or keep them all, depending on your particular situation. 

`pbr` is the line where you can import any image. Take a look at our robot's wheel and the floor. Those are being imported by a picture. We designed this picture as well. 

There are two under `<pbr>` called metal and specular. As of right now, specular is a bug which is why I am using metal to import the picture.

## An Explanation For Sensors:

There are a lot of things to cover about joint_controller and sensors. Unfortunately, since the Ignition is still new, there are currently very limited resources on this topic. As a result, we have been required to think outside the box. 

The ultrasonic in our robot is actually using Lidar. There is no official ultrasonic plugin. In order to accomplish this, I had to replace the few settings inside the ultrasonic sdf and make the range to 4 m distance in order to keep the small cone and use the ROS2 feature called sensor_msgs/msg/Range. Our file is using the float32 range. This allows us to read the distance in real time. 

The infrared sensors are actually using an RGB camera. This case is a very unique case since IR is very rare and almost non-existence so I came up with a way to use the RGB camera and convert it to focus on the one pixel which is 1x1. This allows RGB camera to pretend like it is an IR in gazebo! WIth the 1x1, it gives the RGB data like (R,G,B). 

