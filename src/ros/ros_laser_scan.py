
"""
This module needs to be compiled using colcon for ROS2 and is not directly run by FEAGI

Todo: Need to implement an automated method to deploy and compile this method.
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
10.   # In another terminal, source your ros
11. ros2 run py_topic ros_laser_scan


Note: This module is a modified version of the code from OSRF

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""

import sensor_msgs.msg #this is needed to read lidar or any related to lidar.
import rclpy
import zmq
import serial


from time import sleep
from rclpy.node import Node
from sensor_msgs.msg import LaserScan #to call laserscan so it can convert the data or provide the data
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data #this is required to have a full data
from example_interfaces.msg import Int64



#print("Finding the /scann topic..")

# todo: export socket address to config file
socket_address = 'tcp://127.0.0.1:2000'

context = zmq.Context()
socket = context.socket(zmq.PUB)
print("Binding to socket", socket_address)

# todo: Figure a way to externalize the binding port. feagi_configuration.ini captures it on FEAGI side.
socket.bind(socket_address)
serialcomm = serial.Serial('/dev/ttyACM0', 9600) #connect to ardiuno port.
serialcomm.timeout = 1
print("Found the ardiuno board.")
print("Creating the /scann topic..")


class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('minimal_publisher')
		self.publisher_  = self.create_publisher(
			Int64,
			"scann",
			10)
		timer_period = 0  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		#self.subscription  # prevent unused variable warning

	def timer_callback(self):
		msg = int()
		data_chunk = serialcomm.readline().decode('ascii')
		msg = self.get_logger().info("I heard: {}".format(serialcomm.readline().decode('ascii')))
		#msg.data = self.get_logger().info("I heard: {}").format(serialcomm.readline().decode('ascii'))
		#test=str()
		#test=test.info("I heard: {}".format(serialcomm.readline().decode('ascii')))
		print(type(msg))
		self.publisher_.publish(msg)
		socket.send_pyobj(msg)
		#self.get_logger().info('Publishing: "%s"' % msg.data)
		self.i += 1

#	def listener_callback(self, msg):
#		# self.get_logger().info("I heard: {}".format(msg)) #put .format(msg) to display the data
#		self.get_logger().info("I heard:".serialcomm.readline().decode('ascii'))
#		#self.get_logger().info(serialcomm.readline().decode('ascii'))
#		print("test")
#		socket.send_pyobj(msg)


def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)


	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()
	serialcomm.close()


if __name__ == '__main__':
	main()
