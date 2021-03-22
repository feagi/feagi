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
This module needs to be compiled using colcon for ROS2.

Todo: Need to implement an automated method to deploy and compile this method.

"""

import sensor_msgs.msg #this is needed to read lidar or any related to lidar.
import rclpy
import zmq

from time import sleep
from rclpy.node import Node
from sensor_msgs.msg import LaserScan #to call laserscan so it can convert the data or provide the data
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data #this is required to have a full data

context = zmq.Context()
socket = context.socket(zmq.PUB)

# todo: move the binding port to the feagi_configuration.ini
socket.bind('tcp://127.0.0.1:2000')


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("I heard: {}".format(msg)) #put .format(msg) to display the data
        socket.send_pyobj(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
