
Placeholder for ROS related documentation


ros2 run rqt_graph rqt_graph


1. mkdir -p ~/ros2_ws/src     # Create a ros2 workspace
2. cd ~/ros2_ws/src
3. ros2 pkg create --build-type ament_python py_topic    # Create a ros2 package
4. cp /src/ipu/source/ros/ros_laser_scan.py ~/ros2_ws/src/py_topic/py_topic/
5. edit ~/ros2_ws/src/py_topic/package.xml and add the following 3 lines after the license declaration line
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

6. Edit ~/ros2_ws/src/py_topic/setup.py entry point as follows:
    'console_scripts': ['py_laser_scan = py_topic.ros_laser_scan:main']

7. cd ~/ros2_ws
8. colcon build
9. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  #in one terminal run Turtlebot3
10.  . # In another terminal, source your ros
11. ros2 run py_topic ros_laser_scan

