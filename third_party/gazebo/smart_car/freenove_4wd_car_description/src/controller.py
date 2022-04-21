#!/usr/bin/env python3

# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
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
# ==============================================================================

import sys
import traceback
from datetime import datetime
import router
from threading import Thread
import math
import geometry_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, BatteryState
from rclpy.qos import qos_profile_sensor_data
from configuration import *
import configuration
from configuration import message_to_feagi
from time import sleep
import os

runtime_data = {
    "current_burst_id": 0,
    "feagi_state": None,
    "cortical_list": (),
    "battery_charge_level": 1,
    'motor_status': {},
    'servo_status': {}
}


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


os.system('ign topic -t "/S0" -m ignition.msgs.Double -p "data: 1.6" && ign topic -t "/S1" -m ignition.msgs.Double -p "data: 1.6"')


def feagi_registration(feagi_host, api_port):
    app_host_info = router.app_host_info()
    runtime_data["host_network"]["host_name"] = app_host_info["host_name"]
    runtime_data["host_network"]["ip_address"] = app_host_info["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("Awaiting registration with FEAGI...")
        try:
            runtime_data["feagi_state"] = router.register_with_feagi(app_name=configuration.app_name,
                                                                     feagi_host=feagi_host,
                                                                     api_port=api_port,
                                                                     app_capabilities=configuration.capabilities,
                                                                     app_host_info=runtime_data["host_network"]
                                                                     )
        except:
            pass
        sleep(1)


def block_to_array(block_ref):
    block_id_str = block_ref.split('-')
    array = [int(x) for x in block_id_str]
    return array


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
            compose_message_to_feagi(original_message=formatted_msg)
            self.counter += 1
        except Exception as e:
            print("Error in listener callback...", e)
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)

    @staticmethod
    def msg_processor(msg, msg_type):
        # TODO: give each subclass a specific msg processor method?
        # TODO: add an attribute that explicitly defines message type (instead of parsing topic name)?
        if 'ultrasonic' in msg_type and msg.ranges[1]:
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

            # print("\n***\nAverage Intensity = ", avg_intensity)
            if avg_intensity > 25:
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
        # elif 'battery' in msg_type and msg.percentage:
        #     return {
        #         'battery': {
        #             idx: val for idx, val in enumerate([msg.percentage])
        #         }
        #     }


def compose_message_to_feagi(original_message):
    """
    accumulates multiple messages in a data structure that can be sent to feagi
    """

    if "data" not in message_to_feagi:
        message_to_feagi["data"] = dict()
    if "sensory_data" not in message_to_feagi["data"]:
        message_to_feagi["data"]["sensory_data"] = dict()
    for sensor in original_message:
        if sensor not in message_to_feagi["data"]["sensory_data"]:
            message_to_feagi["data"]["sensory_data"][sensor] = dict()
        for sensor_data in original_message[sensor]:
            if sensor_data not in message_to_feagi["data"]["sensory_data"][sensor]:
                message_to_feagi["data"]["sensory_data"][sensor][sensor_data] = original_message[sensor][sensor_data]
    message_to_feagi["data"]["sensory_data"]["battery"] = {1: runtime_data["battery_charge_level"] / 100}


class UltrasonicSubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class BatterySubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class IRSubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class Battery:
    def __init__(self):
        print("Battery has been initialized")
        if "battery" in capabilities:
            runtime_data["battery_charge_level"] = capabilities["battery"]["capacity"]

    @staticmethod
    def charge_battery():
        print("Charging battery    ^^^^^^^^^^^^   *************    ^^^^^^^^^^^^^^^^^^^")
        runtime_data["battery_charge_level"] += capabilities["battery"]["charge_increment"]
        if runtime_data["battery_charge_level"] > capabilities["battery"]["capacity"]:
            runtime_data["battery_charge_level"] = capabilities["battery"]["capacity"]

    @staticmethod
    def consume_battery():
        # print("Consuming battery ")
        runtime_data["battery_charge_level"] -= capabilities["battery"]["depletion_per_burst"]


class Motor:
    def __init__(self, count, identifier, model):
        self.motor_node = publisher_initializer(model_name=model, topic_count=count, topic_identifier=identifier)

        # todo: figure a way to extract wheel parameters from the model
        self.wheel_diameter = 1

    def move(self, feagi_device_id, power):
        try:
            if feagi_device_id > 2 * capabilities['motor']['count']:
                print("Warning! Number of motor channels from FEAGI exceed available device count!")
            # Translate feagi_device_id to device backward and forward motion to individual devices
            device_index = feagi_device_id // 2
            if feagi_device_id % 2 == 1:
                power *= -1

            device_position = std_msgs.msg.Float64()

            if device_index not in runtime_data['motor_status']:
                runtime_data['motor_status'][device_index] = 0

            device_current_position = runtime_data['motor_status'][device_index]
            device_position.data = float((power * network_settings['feagi_burst_speed']*1.5) + device_current_position)

            runtime_data['motor_status'][device_index] = device_position.data
            # print("device index, position, power = ", device_index, device_position.data, power)
            self.motor_node[device_index].publish(device_position)
        except Exception:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)


class Servo:
    def __init__(self, count, identifier, model):
        self.servo_node = publisher_initializer(model_name=model,
                                                topic_count=count,
                                                topic_identifier=identifier)
        self.device_position = std_msgs.msg.Float64()
        # todo: figure a way to extract servo parameters from the model
        self.servo_ranges = {0: [0, 3.14],
                             1: [0, 3.14]}

    def set_default_position(self):
        try:
            # Setting the initial position for the servo
            servo_0_initial_position = float(1.5708)
            self.device_position.data = servo_0_initial_position
            self.servo_node[0].publish(self.device_position)
            runtime_data['servo_status'][0] = self.device_position.data
            print("Servo 0 was moved to its initial position")

            servo_1_initial_position = float(1.3)
            self.device_position.data = servo_1_initial_position
            self.servo_node[1].publish(self.device_position)
            runtime_data['servo_status'][1] = self.device_position.data
            print("Servo 1 was moved to its initial position")
        except Exception as e:
            print("Error while setting initial position for the servo:", e)

    def keep_boundaries(self, device_id, current_position):
        """
        Prevent Servo position to go beyond range
        """
        if current_position > self.servo_ranges[device_id][1]:
            adjusted_position = float(self.servo_ranges[device_id][1])
        elif current_position < self.servo_ranges[device_id][0]:
            adjusted_position = float(self.servo_ranges[device_id][0])
        else:
            adjusted_position = float(current_position)
        return adjusted_position

    def move(self, feagi_device_id, power):
        try:
            if feagi_device_id > 2 * capabilities['servo']['count']:
                print("Warning! Number of servo channels from FEAGI exceed available Motor count!")
            # Translate feagi_motor_id to motor backward and forward motion to individual motors
            device_index = feagi_device_id // 2
            if feagi_device_id % 2 == 1:
                power *= -1

            if device_index not in runtime_data['servo_status']:
                runtime_data['servo_status'][device_index] = device_index

            device_current_position = runtime_data['servo_status'][device_index]
            # print("servo ", device_index, device_current_position)
            self.device_position.data = float((power * network_settings['feagi_burst_speed'] / 20) +
                                              device_current_position)

            self.device_position.data = self.keep_boundaries(device_id=device_index,
                                                             current_position=self.device_position.data)

            runtime_data['servo_status'][device_index] = self.device_position.data
            # print("device index, position, power = ", device_index, self.device_position.data, power)
            self.servo_node[device_index].publish(self.device_position)
        except Exception:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)
        
        # twist = geometry_msgs.msg.Twist()
        #
        # # todo: linear and angular speed does not make sense in the case of servo. To be fixed
        # twist.linear.x = angle  # positive goes backward, negative goes forward
        # twist.angular.z = angle
        #
        # self.servo_node[servo_index].publish(twist)
        # twist = std_msgs.msg.Float64()
        # degree = angle * 3.14159265359 / 180
        # twist.data = float(degree)
        # self.servo_node[servo_index].publish(twist)  # -1.6 to 1.6 which is -90 to 90


# class Teleop:
#     def __init__(self):
#         rclpy.init()
#         node = rclpy.create_node('teleop_twist_keyboard')
#         self.pub = node.create_publisher(geometry_msgs.msg.Twist, '/model/vehicle_green/cmd_vel', 10)
#         print("Gazebo Teleop has been initialized...")
#
#     def backward(self):
#         twist = geometry_msgs.msg.Twist()
#         twist.linear.x = 2.0  # positive goes backward, negative goes forward
#         twist.angular.z = 0.0
#         print("Backward.")
#         self.pub.publish(twist)
#         sleep(0.5)
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.pub.publish(twist)
#
#     def forward(self):
#         twist = geometry_msgs.msg.Twist()
#         twist.linear.x = -2.0  # positive goes backward, negative goes forward
#         twist.angular.z = 0.0
#         print("Forward.")
#         self.pub.publish(twist)
#         sleep(0.5)
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.pub.publish(twist)
#
#     def left(self):
#         twist = geometry_msgs.msg.Twist()
#         twist.linear.x = 0.0  # positive goes backward, negative goes forward
#         twist.angular.z = 9.0
#         print("Left.")
#         self.pub.publish(twist)
#         sleep(0.5)
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.pub.publish(twist)
#
#     def right(self):
#         twist = geometry_msgs.msg.Twist()
#         twist.linear.x = 0.0  # positive goes backward, negative goes forward
#         twist.angular.z = -9.0
#         print("Right.")
#         self.pub.publish(twist)
#         sleep(0.5)
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.pub.publish(twist)
#

def main(args=None):
    print("Connecting to FEAGI resources...")

    address = 'tcp://' + network_settings['feagi_ip'] + ':' + network_settings['feagi_outbound_port']

    feagi_host = configuration.network_settings["feagi_host"]
    api_port = configuration.network_settings["feagi_api_port"]

    feagi_registration(feagi_host=feagi_host, api_port=api_port)

    print("** **", runtime_data["feagi_state"])
    sockets = runtime_data["feagi_state"]['sockets']
    network_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_frequency'])

    print("--->> >> >> ", sockets)

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = 'tcp://0.0.0.0:' + network_settings['feagi_inbound_port_gazebo']
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = 'tcp://' + network_settings['feagi_ip'] + ':' + sockets['feagi_outbound_port']

    feagi_ipu_channel = router.Pub(address=ipu_channel_address)
    feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)

    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate controller classes with Publisher nature
    motor = Motor(count=capabilities['motor']['count'], identifier=capabilities['motor']['topic_identifier'],
                  model='freenove_motor')
    servo = Servo(count=capabilities['servo']['count'], identifier=capabilities['servo']['topic_identifier'],
                  model='freenove_servo')

    # Instantiate controller classes with Subscriber nature

    # Ultrasonic Sensor
    ultrasonic_feed = UltrasonicSubscriber('ultrasonic0', LaserScan, 'ultrasonic0')
    executor.add_node(ultrasonic_feed)

    # Battery
    # Moving away from using the Gazebo based battery model and adopting a controller based model instead
    # battery_feed = BatterySubscriber('battery', BatteryState, 'model/freenove_smart_car/battery/linear_battery/state')
    # # todo: Change the topic name and make it scalable
    # executor.add_node(battery_feed)
    battery = Battery()

    # Infrared Sensor
    ir_feeds = {}
    ir_topic_id = capabilities['infrared']['topic_identifier']
    for ir_node in range(capabilities['infrared']['count']):
        ir_feeds[ir_node] = IRSubscriber(f'infrared_{ir_node}', Image, f'{ir_topic_id}{ir_node}/image')
        executor.add_node(ir_feeds[ir_node])

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    msg_counter = 0

    # Positioning servos to a default position
    servo.set_default_position()

    try:
        while True:
            # Process OPU data received from FEAGI and pass it along
            message_from_feagi = feagi_opu_channel.receive()
            battery.consume_battery()
            try:
                opu_data = message_from_feagi["opu_data"]
                print("Received:", opu_data)
                if opu_data is not None:
                    if 'o__mot' in opu_data:
                        for data_point in opu_data['o__mot']:
                            data_point = block_to_array(data_point)
                            device_id = data_point[0]
                            device_power = data_point[2]
                            motor.move(feagi_device_id=device_id, power=device_power)
                    if 'o__ser' in opu_data:
                        for data_point in opu_data['o__ser']:
                            data_point = block_to_array(data_point)
                            device_id = data_point[0]
                            device_power = data_point[2]
                            servo.move(feagi_device_id=device_id, power=device_power)
                    if 'o__bat' in opu_data:
                        battery.charge_battery()

            except Exception:
                pass
                #print("")
            message_to_feagi['timestamp'] = datetime.now()
            message_to_feagi['counter'] = msg_counter
            feagi_ipu_channel.send(message_to_feagi)
            message_to_feagi.clear()
            msg_counter += 1
            sleep(network_settings['feagi_burst_speed'])
    except KeyboardInterrupt:
        pass

    ultrasonic_feed.destroy_node()
    # battery_feed.destroy_node()

    for ir_node in ir_feeds:
        ir_feeds[ir_node].destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

