import rclpy
import serial
import time
import std_msgs
import zmq

from example_interfaces.msg import Int64
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data  # this is required to have a full data

ser = serial.Serial(
    port="/dev/ttyACM0",
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)  # connect to ardiuno port.
# serialcomm.timeout = 1
print("Found the ardiuno board.")
print("Creating the /scan topic..")

socket_address = "tcp://0.0.0.0:2000"
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(socket_address)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int64, "scan", 10)
        timer_period = 0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        bytes = ser.readline()
        data = bytes.decode(encoding="utf-8").strip("\r\n")
        distance = int(data)
        self.get_logger().info("MESSAGE: {}".format(distance))
        socket.send_pyobj(distance)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    # serialcomm.close()


if __name__ == '__main__':
    main()

