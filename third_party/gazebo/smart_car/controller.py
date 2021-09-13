"""
This controller module is responsible for interacting directly with virtual devices in Gazebo
"""
#!/usr/bin/env python3

import sys
import time

import geometry_msgs.msg
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class Ultrasonic:
    def __init__(self):
        print("Gazebo Ultrasonic has been initialized...")

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

    def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


class Teleop:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node('teleop_twist_keyboard')
        pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)
        print("Gazebo Teleop has been initialized...")

    def backward():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 2.0  # positive goes backward, negative goes forward
        twist.angular.z = 0.0
        print("Backward.")
        pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    def forward():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = -2.0  # positive goes backward, negative goes forward
        twist.angular.z = 0.0
        print("Forward.")
        pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    def left():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0  # positive goes backward, negative goes forward
        twist.angular.z = 9.0
        print("Left.")
        pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    def right():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0  # positive goes backward, negative goes forward
        twist.angular.z = -9.0
        print("Right.")
        pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)


class Motor:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node('Controller_py')
        M4 = node.create_publisher(geometry_msgs.msg.Twist, '/M4', 10)
        M3 = node.create_publisher(geometry_msgs.msg.Twist, '/M3', 10)
        M2 = node.create_publisher(geometry_msgs.msg.Twist, '/M2', 10)
        M1 = node.create_publisher(geometry_msgs.msg.Twist, '/M1', 10)
        servo = node.create_publisher(geometry_msgs.msg.Twist, '/servo', 10)
        servo1 = node.create_publisher(geometry_msgs.msg.Twist, '/servo1', 10)
        pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)
        pub1 = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)  # Might delete this line

    def M1T(num, num1):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = num  # positive goes backward, negative goes forward
        twist.angular.z = num1
        M1.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M1.publish(twist)

    def M2T(num, num1):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = num  # positive goes backward, negative goes forward
        twist.angular.z = num1
        M2.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M2.publish(twist)

    def M3T(num, num1):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = num  # positive goes backward, negative goes forward
        twist.angular.z = num1
        M3.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M3.publish(twist)

    def M4T(num, num1):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = num  # positive goes backward, negative goes forward
        twist.angular.z = num1
        M4.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M4.publish(twist)

    def setMotorModel(speed, speed1):
        """

        Parameters
        ----------
        self
        speed = -10 to 10 to move forward/backward. Negative is backward at the moment.
        speed1 = -10 to 10 to move right/left. Negative is left at the moment.

        Returns
        -------

        """
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = speed  # positive goes backward, negative goes forward
        twist.angular.z = speed1
        M1T(speed, speed1)
        M2T(speed, speed1)
        M3T(speed, speed1)
        M4T(speed, speed1)

    def backward():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.5  # positive goes backward, negative goes forward
        twist.angular.z = 0.0
        print("Backward.")
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)

    def forward():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = -0.5  # positive goes backward, negative goes forward
        twist.angular.z = 0.0
        print("Forward.")
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)

    def left():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0  # positive goes backward, negative goes forward
        twist.angular.z = 4.0
        print("Left.")
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)

    def right():
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0  # positive goes backward, negative goes forward
        twist.angular.z = -4.0
        print("Right.")
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        M1.publish(twist)
        M2.publish(twist)
        M4.publish(twist)
        M3.publish(twist)

    def head_UP_DOWN(num, num1):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = num  # positive goes backward, negative goes forward
        twist.angular.z = num1
        print("HEAD_UP_DOWN")
        servo.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        servo.publish(twist)

    def head_RIGHT_LEFT(num, num1):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = num  # positive goes backward, negative goes forward
        twist.angular.z = num1
        print("HEAD_RIGHT_LEFT")
        servo1.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        servo1.publish(twist)
