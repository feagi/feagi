#!/usr/bin/env python3

import sys
import time

import geometry_msgs.msg
import std_msgs.msg
import rclpy
import math

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

rclpy.init()
node = rclpy.create_node('Controller_py')
# M3 = node.create_publisher(geometry_msgs.msg.Twist, '/M3', 10)
# M2 = node.create_publisher(geometry_msgs.msg.Twist, '/M2', 10)
# M1 = node.create_publisher(geometry_msgs.msg.Twist, '/M1', 10)
# M0 = node.create_publisher(geometry_msgs.msg.Twist, '/M0', 10)
servo = node.create_publisher(geometry_msgs.msg.Twist, '/servo', 10)
servo1 = node.create_publisher(geometry_msgs.msg.Twist, '/servo1', 10)
pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)
pub1 = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)  # Might delete this line
M0 = node.create_publisher(std_msgs.msg.Float64, '/M0_controller', 10)
M1 = node.create_publisher(std_msgs.msg.Float64, '/M1_controller', 10)
M2 = node.create_publisher(std_msgs.msg.Float64, '/M2_controller', 10)
M3 = node.create_publisher(std_msgs.msg.Float64, '/M3_controller', 10)
S0 = node.create_publisher(std_msgs.msg.Float64, '/S0_controller', 10)
try:
    bank_number = bank_number
except NameError:
    bank_number = 0.0


def increment_wheel(random_number):
    global bank_number
    value = std_msgs.msg.Float64()
    #value.data = float(random_number)
    print(bank_number)
    hold_number = bank_number
    print("input: ")
    print(random_number)
    checkpoint = 0.0
    if random_number > 0:
        while random_number > checkpoint:
            hold_number = hold_number + (random_number - random_number + 0.5)
            value.data = hold_number
            M0.publish(value)
            M1.publish(value)
            M2.publish(value)
            M3.publish(value)
            print(hold_number)
            checkpoint = checkpoint + 0.1
            time.sleep(0.1)
    elif random_number < 0:
        print("negative started")
        print(checkpoint)
        while random_number < checkpoint:
            hold_number = hold_number + (random_number - random_number - 0.5)
            print("line 78 of random")
            print(random_number - random_number - 0.1)
            value.data = hold_number
            M0.publish(value)
            M1.publish(value)
            M2.publish(value)
            M3.publish(value)
            print(hold_number)
            checkpoint = checkpoint - 0.1
            time.sleep(0.1)
    print("bank: ")
    print(bank_number)
    bank_number = hold_number

def sample(random_number):
    value = std_msgs.msg.Float64()
    degree = random_number * 3.14159265359/180
    value.data = float(degree)
    S0.publish(value) #-1.6 to 1.6 which is -90 to 90


if __name__ == '__main__':
    while True:
        print("Value: ")
        increment_wheel(float(input()))
