"""

"""

from time import sleep
import zmq
import controller
from configuration import controller_settings


class Pub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)
        print("<< Incomplete Code >>")


class Sub:
    def __init__(self, address, flags=None):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))
        self.flag = flags

    @staticmethod
    def validate(payload):
        """
        This function endures the received payload meets a certain expectations
        """
        try:
            # todo: define validation criterias
            # print("<< Incomplete TRY Code >>")
            return True
        except:
            print("<< Incomplete EXCEPTION Code >>")
            return False

    def receive(self):
        try:
            payload = self.socket.recv_pyobj(self.flag)

            if self.validate(payload):
                return payload

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass
            else:
                print(e)


def find_feagi():
    print('Awaiting connection with FEAGI...')
    subscriber = Sub(address=controller_settings['sockets']['general']['burst_beacon'], flags=zmq.SUB)
    message = subscriber.receive()
    print("Connection to FEAGI has been established")

    # todo: What information is useful to receive from FEAGI in this message? IPU/OPU list?
    print("Current FEAGI state is at burst number ", message)


def register_with_feagi():
    print("Registering router with FEAGI")
    publisher_ = Pub('tcp://0.0.0.0:11000')

    # todo: need to send a set of capabilities to FEAGI
    publisher_.send(message={"A", "Hello!"})

    print("Router registration has successfully completed!")


if __name__ == '__main__':
    find_feagi()
    register_with_feagi()

    # todo: to obtain this info directly from FEAGI as part of registration
    opu_channel_address = controller_settings['sockets']['general']['opu_channel']
    feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)

    print("Connecting to FEAGI resources...")

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate Controller Classes
    motor = controller.Motor()
    # ir = controller.IR()

    # Listen and route
    print("Starting the routing engine for ", controller_settings['properties']['mode'])
    print("Communication frequency is set once every %f seconds" % controller_settings['timers']['global_timer'])

    while True:
        if controller_settings['properties']['mode'] == 'rpi':
            print("<< Incomplete RPI Code >>")

        elif controller_settings['properties']['mode'] == 'virtual':
            # Process OPU data received from FEAGI and pass it along to the controller.py
            opu_data = feagi_opu_channel.receive()
            print(opu_data)
            if opu_data is not None:
                if 'motor' in opu_data:
                    for motor_id in opu_data['motor']:
                        motor.move(motor_id, opu_data['motor'][motor_id])

            # Process IPU data received from controller.py and pass it along to FEAGI
            ipu_data = dict()
            ipu_data['ultrasonic'] = {}
            ipu_data['ir'] = {}

            # todo: need to figure how to correlate the flow on incoming data with the rate data is passed to FEAGI





        elif controller_settings['properties']['mode'] == 'ros':
            # Cycle through all subscribed ROS topics and publish them to the corresponding FEAGI channel
            print("<< Incomplete ROS Code >>")

        # Cycle through all FEAGI messages and publish them on the corresponding ROS channel
            print("<< Incomplete Code >>")

        else:
            print("Error: Chosen controller mode is not supported!")

        # Cycle through all the

        sleep(controller_settings['timers']['global_timer'])
