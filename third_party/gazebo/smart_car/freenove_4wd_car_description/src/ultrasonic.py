#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from rclpy.qos import qos_profile_sensor_data

from configuration import *
from router import *


address = 'tcp://' + router_settings['feagi_ip'] + ':' + router_settings['feagi_port']
feagi_state = find_feagi(address=address)

print("** **", feagi_state)
sockets = feagi_state['sockets']

print("--->> >> >> ", sockets)

# todo: to obtain this info directly from FEAGI as part of registration
ipu_channel_address = 'tcp://0.0.0.0:' + router_settings['ipu_port']
print("IPU_channel_address=", ipu_channel_address)

feagi_ipu_channel = Pub(address=ipu_channel_address)


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
            self.send_to_feagi(message=formatted_msg)
        except Exception as e:
            print(">>>>>>>>>>>> ERROR: ", e)

    def format_ultrasonic_msg(self, msg, sensor_type='ultrasonic'):
        return {
            sensor_type: {
                idx: val for idx, val in enumerate(msg)
            }
        }

    def send_to_feagi(self, message):
        print("Sending message to FEAGI...")
        print("Original message:", message)

        feagi_ipu_channel.send(message)


def main(args=None):
    rclpy.init(args=args)

    ultrasonic_sub = UltraSonicSubscriber()

    rclpy.spin(ultrasonic_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ultrasonic_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
