#!/usr/bin/env python3

import sys
import time
import traceback
from datetime import datetime
from router import *
from random import randrange
from threading import Thread

import geometry_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
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
router_settings['feagi_burst_speed'] = float(feagi_state['burst_frequency'])

print("--->> >> >> ", sockets)

# todo: to obtain this info directly from FEAGI as part of registration
ipu_channel_address = 'tcp://0.0.0.0:' + router_settings['ipu_port']
print("IPU_channel_address=", ipu_channel_address)
opu_channel_address = 'tcp://' + router_settings['feagi_ip'] + ':' + sockets['opu_port']

feagi_ipu_channel = Pub(address=ipu_channel_address)
feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)


def publisher_initializer(model_name, topic_count, topic_identifier):
    node = rclpy.create_node('Controller_py')

    target_node = {}
    for target in range(topic_count):
        topic_string = topic_identifier + str(target)
        target_node[target] = node.create_publisher(std_msgs.msg.Float64, topic_string, 10)

    # is this used for anything?
    node.create_publisher(geometry_msgs.msg.Twist, model_name, 10)

    return target_node


class ScalableSubscriber(Node):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name)
        self.subscription = self.create_subscription(
            msg_type,
            topic,
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.topic = topic
        self.counter = 0

    def listener_callback(self, msg):
        # self.get_logger().info("Raw Message: {}".format(msg))
        try:
            formatted_msg = self.msg_processor(msg, self.topic)
            send_to_feagi(message=formatted_msg, counter=self.counter)
            self.counter += 1
        except Exception as e:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)

    def msg_processor(self, msg, msg_type):
        # TODO: give each subclass a specific msg processor method?
        # TODO: add an attribute that explicitly defines message type (instead of parsing topic name)?
        if 'ultrasonic' in msg_type:
            return {
                msg_type: {
                    idx: val for idx, val in enumerate([msg.ranges[1]])
                }
            }
        elif 'IR' in msg_type:
            rgb_vals = list(msg.data)
            avg_intensity = sum(rgb_vals) // len(rgb_vals)

            sensor_topic = msg_type.split('/')[0]
            sensor_id = int(''.join(filter(str.isdigit, sensor_topic)))

            if avg_intensity < 100:
                return {
                    'ir': {
                        sensor_id: False
                    }
                }
            else:
                return {
                    'ir': {
                        sensor_id: True
                    }
                }


class UltrasonicSubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class IRSubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class Motor:
    def __init__(self, count, identifier, model):
        self.motor_node = publisher_initializer(model_name=model, topic_count=count, topic_identifier=identifier)

        # todo: figure a way to extract wheel parameters from the model
        self.wheel_diameter = 1

    def move(self, motor_index, speed):
        try:
            motor_position = std_msgs.msg.Float64()
            try:
                motor_current_position = model_properties['motor']['motor_statuses'][motor_index]
                motor_position.data = float(speed * router_settings['feagi_burst_speed'] / 10000 + motor_current_position)
            except KeyError:
                model_properties['motor']['motor_statuses'][motor_index] = 0
                motor_position.data = float(speed * router_settings['feagi_burst_speed'] / 10000)

            model_properties['motor']['motor_statuses'][motor_index] += motor_position.data
            print("Motor index + position = ", motor_index, motor_position.data)
            self.motor_node[motor_index].publish(motor_position.data)
        except Exception:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)


class Servo:
    def __init__(self, count, identifier, model):
        self.servo_node = publisher_initializer(model_name=model,
                                                topic_count=count,
                                                topic_identifier=identifier)

        # todo: figure a way to extract servo parameters from the model
        self.servo_range = [0, 180]

    def move(self, servo_index, angle):
        # twist = geometry_msgs.msg.Twist()
        #
        # # todo: linear and angular speed does not make sense in the case of servo. To be fixed
        # twist.linear.x = angle  # positive goes backward, negative goes forward
        # twist.angular.z = angle
        #
        # self.servo_node[servo_index].publish(twist)
        twist = std_msgs.msg.Float64()
        degree = angle * 3.14159265359 / 180
        twist.data = float(degree)
        self.servo_node[servo_index].publish(twist)  # -1.6 to 1.6 which is -90 to 90


class Teleop:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node('teleop_twist_keyboard')
        self.pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)
        print("Gazebo Teleop has been initialized...")

    def backward(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 2.0  # positive goes backward, negative goes forward
        twist.angular.z = 0.0
        print("Backward.")
        self.pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def forward(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = -2.0  # positive goes backward, negative goes forward
        twist.angular.z = 0.0
        print("Forward.")
        self.pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def left(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0  # positive goes backward, negative goes forward
        twist.angular.z = 9.0
        print("Left.")
        self.pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def right(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0  # positive goes backward, negative goes forward
        twist.angular.z = -9.0
        print("Right.")
        self.pub.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)


def send_to_feagi(message, counter):
    print("Sending message to FEAGI...")
    print("Original message:", message)

    # pause before sending to FEAGI IPU SUB (avoid losing connection)
    time.sleep(router_settings['feagi_burst_speed'])
    message['timestamp'] = datetime.now()
    message['counter'] = counter
    feagi_ipu_channel.send(message)


def main(args=None):
    print("Connecting to FEAGI resources...")
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate controller classes with Publisher nature
    motor = Motor(count=model_properties['motor']['count'], identifier=model_properties['motor']['topic_identifier'],
                  model='freenove_motor')
    servo = Servo(count=model_properties['servo']['count'], identifier=model_properties['servo']['topic_identifier'],
                  model='freenove_servo')

    # Instantiate controller classes with Subscriber nature
    ultrasonic_feed = UltrasonicSubscriber('ultrasonic0', LaserScan, 'ultrasonic0')
    executor.add_node(ultrasonic_feed)

    ir_feeds = {}
    ir_topic_id = model_properties['infrared']['topic_identifier']
    for ir_node in range(model_properties['infrared']['count']):
        ir_feeds[ir_node] = IRSubscriber(f'infrared_{ir_node}',
                                        Image,
                                        f'{ir_topic_id}{ir_node}/image')
        executor.add_node(ir_feeds[ir_node])

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while True:
            # Process OPU data received from FEAGI and pass it along
            opu_data = feagi_opu_channel.receive()
            print("Received:", opu_data)
            if opu_data is not None:
                if 'motor' in opu_data:
                    for motor_id in opu_data['motor']:
                        motor.move(motor_index=motor_id, speed=opu_data['motor'][motor_id])

            time.sleep(router_settings['feagi_burst_speed'])
    except KeyboardInterrupt:
        pass

    ultrasonic_feed.destroy_node()

    for ir_node in ir_feeds:
        ir_feeds[ir_node].destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
