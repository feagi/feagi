#!/usr/bin/env python3

import sys
import time
from router import *
from random import randrange, getrandbits

import geometry_msgs.msg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
# from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from configuration import *


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

address = 'tcp://' + router_settings['feagi_ip'] + ':' + router_settings['feagi_port']
feagi_state = find_feagi(address=address)

print("** **", feagi_state)
sockets = feagi_state['sockets']

print("--->> >> >> ", sockets)

# todo: to obtain this info directly from FEAGI as part of registration
ipu_channel_address = 'tcp://0.0.0.0:' + router_settings['ipu_port']
print("IPU_channel_address=", ipu_channel_address)
opu_channel_address = 'tcp://' + router_settings['feagi_ip'] + ':' + sockets['opu_port']

feagi_ipu_channel = Pub(address=ipu_channel_address)
feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)


def publisher_initializer(model_name, topic_count, topic_identifier):
    rclpy.init()
    node = rclpy.create_node('Controller_py')

    target_node = []
    for target in range(topic_count):
        topic_string = topic_identifier + str(target)
        target_node[target] = node.create_publisher(geometry_msgs.msg.Twist, topic_string, 10)

    pub = node.create_publisher(geometry_msgs.msg.Twist, model_name, 10)
    # pub1 = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)  # Might delete this line

    return target_node


class ScalableSubscriber(Node):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name)
        self.subscription = self.create_subscription(
            msg_type,
            topic,
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.msg_type = msg_type

    def listener_callback(self, msg):
        self.get_logger().info("Raw Message: {}".format(msg.ranges[1]))
        try:
            formatted_msg = self.msg_processor(msg, self.msg_type)
            print(">>>>>>>>>>>> MSG (formatted): ", formatted_msg)
            send_to_feagi(message=formatted_msg)
        except Exception as e:
            print(">>>>>>>>>>>> ERROR: ", e)

    def msg_processor(self, msg, sensor_type):
        if sensor_type == 'ultrasonic':
            return {
                sensor_type: {
                    idx: val for idx, val in enumerate(msg.ranges[1])
                }
            }
        elif sensor_type == 'infrared':
            return {
                # todo: need a different processing for IR data since its an image
                sensor_type: {
                    idx: val for idx, val in enumerate(msg)
                }
            }



class UltrasonicSubscriber(ScalableSubscriber):
    def __init__(self):
        super().__init__(, )



class UltraSonicSubscriber(Node):
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
        try:
            formatted_msg = self.format_ultrasonic_msg([msg.ranges[1]])
            print(">>>>>>>>>>>> MSG (formatted): ", formatted_msg)
            send_to_feagi(message=formatted_msg)
        except Exception as e:
            print(">>>>>>>>>>>> ERROR: ", e)

    def format_ultrasonic_msg(self, msg, sensor_type='ultrasonic'):
        return {
            sensor_type: {
                idx: val for idx, val in enumerate(msg)
            }
        }


class IRSubscriber(Node):
    def __init__(self):
        super().__init__('ir_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'ultrasonic',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("distance: {}".format(msg.ranges[1]))
        try:
            formatted_msg = self.format_ultrasonic_msg([msg.ranges[1]])
            print(">>>>>>>>>>>> MSG (formatted): ", formatted_msg)
            send_to_feagi(message=formatted_msg)
        except Exception as e:
            print(">>>>>>>>>>>> ERROR: ", e)

    def format_ir_msg(self, msg, sensor_type='ultrasonic'):
        return {
            sensor_type: {
                idx: val for idx, val in enumerate(msg)
            }
        }


class Motor:
    def __init__(self, count, identifier, model):

        self.motor_node = publisher_initializer(model_name=model, topic_count=count, topic_identifier=identifier)

        # todo: figure a way to extract wheel parameters from the model
        self.wheel_diameter = 1

    def move(self, motor_index, speed):
        # Convert speed to linear
        # todo: fix the following formula to properly calculate the velocities in 3d axis
        linear_velocity = [speed, 0, 0]
        angular_velocity = [0, 0, speed]

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = linear_velocity[0]  # positive goes backward, negative goes forward
        twist.angular.z = angular_velocity[2]
        self.motor_node[motor_index].publish(twist)


class Servo:
    def __init__(self, count, identifier, model):

        self.servo_node = publisher_initializer(model_name=model,
                                                     topic_count=count,
                                                     topic_identifier=identifier)

        # todo: figure a way to extract servo parameters from the model
        self.servo_range = [0, 180]

    def move(self, servo_index, angle):

        twist = geometry_msgs.msg.Twist()

        # todo: linear and angular speed does not make sense in the case of servo. To be fixed
        twist.linear.x = angle  # positive goes backward, negative goes forward
        twist.angular.z = angle

        self.servo_node[servo_index].publish(twist)


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


def send_to_feagi(message):
    print("Sending message to FEAGI...")
    print("Original message:", message)

    feagi_ipu_channel.send(message)


def ipu_message_builder():
    """
    This function encodes the sensory information in a dictionary that can be decoded on the FEAGI end.

    expected ipu_data structure:

        ipu_data = {
            sensor_type: {
                sensor_name: sensor_data,
                sensor_name: sensor_data,
                ...
                },
            sensor_type: {
                sensor_name: sensor_data,
                sensor_name: sensor_data,
                ...
                },
            ...
            }
        }
    """
    # Process IPU data received from controller.py and pass it along to FEAGI
    # todo: move class instantiations to outside function
    # ir = IR()

    # todo: figure a better way of obtaining the device count
    # ir_count = 3

    ipu_data = dict()
    ipu_data['ultrasonic'] = {
        1: [randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10,
            randrange(0, 30) / 10, randrange(0, 30) / 10]
    }

    # ipu_data['ir'] = {}
    # for _ in range(ir_count):
    #     ipu_data['ir'][_] = ir.read()

    return ipu_data


def main(args=None):
    print("Connecting to FEAGI resources...")

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate controller classes with Publisher nature
    motor = Motor(count=model_properties['motor']['count'], identifier=model_properties['motor']['topic_identifier'])
    servo = Servo(count=model_properties['servo']['count'], identifier=model_properties['servo']['topic_identifier'])

    rclpy.init(args=args)

    # Instantiate controller classes with Subscriber nature
    ultrasonic_feed = UltraSonicSubscriber()
    ir_feed = IRSubscriber()

    rclpy.spin(ultrasonic_feed)

    for ir in ir_list:
        rclpy.spin(ir_feed)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    while True:
        # Process OPU data received from FEAGI and pass it along
        opu_data = feagi_opu_channel.receive()
        print("Received:", opu_data)
        if opu_data is not None:
            if 'motor' in opu_data:
                for motor_id in opu_data['motor']:
                    motor.move(motor_id, opu_data['motor'][motor_id])

        time.sleep(router_settings['global_timer'])

    ultrasonic_feed.destroy_node()
    for ir in ir_list:
        ir_feed.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
