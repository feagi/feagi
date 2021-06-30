import zmq

socket_address = "tcp://talk:21000"

print("Attempting to subscribe to socket ", socket_address)

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.connect(socket_address)
# socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

# listener = 0

# message = socket.recv_pyobj()
# method_list = [method for method in dir(message) if method.startswith('_') is False]

while True:
    print('****   *****   Anticipating receiving a message   *****   *****')
    message = socket.recv_json()
    print(message, flush=True)
