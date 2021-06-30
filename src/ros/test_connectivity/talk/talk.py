import zmq
import sys
# from rclpy.node import Node

# print("Enter the receiver (FEAGI Container) IP address: ")
# ip_address = input()
#
# print("Enter the receiver (FEAGI Container) port number: ")
# port = input()


ip_address = sys.argv[1]
port = sys.argv[2]

socket_address = "tcp://"+ip_address+":"+port

# socket_address = 'tcp://127.0.0.1:21001'

print('Using %s as socket address' % socket_address)

context = zmq.Context()
socket = context.socket(zmq.PUSH)
print("Binding to socket", socket_address)

socket.bind(socket_address)

try:
    socket.send_json({"ranges": 1})
except Exception as e:
    print(str(e), flush=True)
print("Sending the test message to the destination...")
