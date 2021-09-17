from time import time
import zmq
import controller
from configuration import controller_settings



class Pub:
    def __init__(self, address):


    def send(self, id, message):




class Sub:
    def __init__(self, address):
        # TODO: resolve interface to differentiate between running in container vs locally
        # try:
        #     if os.environ['CONTAINERIZED']:
        #         socket_address = f"tcp://{interface}:{port}"
        # except KeyError:
        #     socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

        socket_address = controller_settings['ultrasonic']

        print("Attempting to subscribe to socket ", socket_address)

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(socket_address)
        self.socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    @staticmethod
    def validate(payload):
        """
        This function endures the received payload meets a certain expectations
        """
        try:
            # todo
            return True
        except:
            # todo
            return False



    def receive(self):
        payload = self.socket.recv_pyobj()

        if self.validate(payload):
            return payload


if __name__ == '__main__':

    print("Connecting to FEAGI resources...")

    # Establish the needed zmq connections for FEAGI communications
    for socket_address in controller_settings['FEAGI_sockets']['pub']:
        publisher = Pub(address=socket_address)
        print("   Connection to %s has been established.", socket_address)

    for socket_address in controller_settings['FEAGI_sockets']['sub']:
        subscriber = Sub(address=socket_address)
        print("   Connection to %s has been established.", socket_address)


    # Listen and route
    print("Starting the routing engine for ", controller_settings['properties']['mode'])
    print("Communication frequency is set once every %i seconds", controller_settings['timers']['global_timer'])
    while True:
        if controller_settings['properties']['mode'] == 'rpi':
            print("<< Incomplete Code >>")


        elif controller_settings['properties']['mode'] == 'virtual':
            print("<< Incomplete Code >>")


        elif controller_settings['properties']['mode'] == 'ros':
        # Cycle through all subscribed ROS topics and publish them to the corresponding FEAGI channel

        # Cycle through all FEAGI messages and publish them on the corresponding ROS channel

        else:
            print("Error: Chosen controller mode is not supported!")

        # Cycle through all the

        time.sleep(controller_settings['timers']['global_timer'])
