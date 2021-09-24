#!/usr/bin/env python3

import sys
import time

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


rclpy.init()
node = rclpy.create_node('teleop_twist_keyboard')
pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)

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


if __name__ == '__main__':
    backward()
    time.sleep(1)
    forward()
    time.sleep(1)
    left()
    time.sleep(1)
    right()
