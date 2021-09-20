#!/usr/bin/env python3

import sys
import time
import traceback

import zmq

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

        # Assuming 4 motors
        # todo: generalize to extract number of motors from gazebo robot model
        motor_count = 4

        self.motor_node = {}
        for motor in range(motor_count):
            motor_string = '/M' + str(motor)
            self.motor_node[motor] = node.create_publisher(geometry_msgs.msg.Twist, motor_string, 10)

        self.pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)
        self.pub1 = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)  # Might delete this line

        # todo: figure a way to extract wheel parameters from the model
        self.wheel_diameter = 1

    def receive_motor_data(self, socket):
        try:
            motor_data = socket.recv_pyobj()
            print(">>>>>>>>>> >>>>>> >>>>>>>>>>> MOTOR DATA RECEIVED: ", motor_data)
            
            (motor_id, motor_speed), = motor_data.items()
            return (motor_id, motor_speed)
        except Exception as e:
            if e.errno == zmq.EAGAIN:
                pass
            else:
                traceback.print_exc()

    def move(self, motor_index, speed):

        # Convert speed to linear
        # todo: fix the following formula to properly calculate the velocities in 3d axis
        linear_velocity = [0, 0, 0]
        angular_velocity = [0, 0, speed]

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = float(linear_velocity[0])  # positive goes backward, negative goes forward
        twist.angular.z = float(angular_velocity[2])
        self.motor_node[motor_index].publish(twist)


class Servo:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node('Controller_py')

        # Assuming 4 motors
        servo_count = 4
        # todo: generalize to support any number of motors
        self.servo_node = []
        for servo in range(servo_count):
            motor_string = '/servo' + str(servo)
            self.motor_node[servo] = node.create_publisher(geometry_msgs.msg.Twist, motor_string, 10)

        servo = node.create_publisher(geometry_msgs.msg.Twist, '/servo', 10)
        servo1 = node.create_publisher(geometry_msgs.msg.Twist, '/servo1', 10)

        self.pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)
        self.pub1 = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)  # Might delete this line

        # todo: figure a way to extract wheel parameters from the model
        self.servo_range = [0, 180]

    def move(self, servo_index, angle):

        twist = geometry_msgs.msg.Twist()

        # todo: linear and angular speed does not make sense in the case of servo. To be fixed
        twist.linear.x = angle  # positive goes backward, negative goes forward
        twist.angular.z = angle

        print("HEAD_UP_DOWN")
        self.servo_node.publish(twist)


if __name__ == '__main__':
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://feagi:2500")
    socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    motor = Motor()

    while True:
        (motor_id, motor_speed) = motor.receive_motor_data(socket)
        # motor_speed += 2
        motor.move(motor_id, float(motor_speed))
        # time.sleep(0.1)
