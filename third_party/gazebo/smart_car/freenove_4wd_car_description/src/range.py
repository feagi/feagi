#!/usr/bin/env python3

import zmq

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


print("Starting FEAGI-ROS Laser Scan Interface...")

socket_address = 'tcp://0.0.0.0:2000'

context = zmq.Context()
socket = context.socket(zmq.PUB)
print("Binding to socket", socket_address)

socket.bind(socket_address)
print("Laser scanner message queue has been activated...")


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'ultrasonic',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("distance: {}".format(msg.ranges[1]))
        print(">>>>>>>>>>>>>>>>>>>> MESSAGE: ", msg.ranges[1])
        try:
            socket.send_pyobj(msg.ranges[1])
        except Exception as e:
            print(">>>>>>>>>>>>>>>>>>>>>>>> ERROR: ", e)


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

