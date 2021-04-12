
***Placeholder for ROS related documentation***

.

*To monitor active topics*

```ros2 run rqt_graph rqt_graph```

---
*To activate FEAGI <> ROS laser_scan interface*

1. ./ros2_ws_setup.sh 
   1. ```mkdir -p ~/ros2_ws/src```     # Create a ros2 workspace
   2. ```cd ~/ros2_ws/src```
   3. ```ros2 pkg create --build-type ament_python py_topic```    # Create a ros2 package
   4. ```cp /src/ipu/source/ros/py_laser_scan.py ~/ros2_ws/src/py_topic/py_topic/```
2. edit ```~/ros2_ws/src/py_topic/package.xml``` and add the following 3 lines after the license declaration line:
```
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
```


3. Edit `````~/ros2_ws/src/py_topic/setup.py````` entry point as follows:
   
       'console_scripts': ['py_laser_scan = py_topic.ros_laser_scan:main']
 
7. ```cd ~/ros2_ws```
8. ```colcon build```
9. in one terminal run Turtlebot3 ```ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py```
10. In another terminal, source your ros
11. start the laser scan topic ```ros2 run py_topic py_laser_scan```


----

*For manually moving the Turtlebot in Gazebo*

```ros2 run turtlebot3_teleop teleop_keyboard```

