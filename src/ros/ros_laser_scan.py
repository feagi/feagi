
import sensor_msgs.msg  # this is needed to read lidar or any related to lidar.
import rclpy
import zmq
import serial
import time
import std_msgs

from example_interfaces.msg import Int64
from time import sleep
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # to call laserscan so it can convert the data or provide the data
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data  # this is required to have a full data

ser = serial.Serial(
	port="/dev/ttyACM0",
	baudrate=9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
)  # connect to ardiuno port.
#serialcomm.timeout = 1
print("Found the ardiuno board.")
print("Creating the /scan topic..")


class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('minimal_publisher')
		self.publisher_ = self.create_publisher(
			Int64,
			"scann",
			10)
		timer_period = 0  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

	# self.subscription  # prevent unused variable warning

	def timer_callback(self): #this is the part where we need to get it keep running
		#print(float(ser.readline())) #to see the actual value converted into a float
		check=ser.readline()
		if check == ' ': #this if statement is to skip string id. It doesn't seem like it works
			print("Skipped the ' '") #in #44 line, it kept recieving a string ' '
		else:
			sensorvalue=float(ser.readline()) #posts the value
		#msg=float()
		msg=sensorvalue
		print(msg)
		#msg.data=self.get_logger().info("distance: ".format(float(ser.readline())))
		#if (float(ser.readline())== ' '):
		#	msg=0 #skipping the string part
		#else:
			#msg = self.get_logger().info("distance: ".format(float(ser.readline())))
		print(type(msg)) # this is to verify the type of the value. It should be float only
		self.publisher_.publish(msg) #this is to publish the data to topic 'scann'. It can change to 'scan' in #34 line
		# self.get_logger().info('Publishing: "%s"' % msg.data)
		self.i += 1


def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()
	serialcomm.close()


if __name__ == '__main__':
	main()
