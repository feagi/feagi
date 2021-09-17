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
    subscriber = Sub(address=controller_settings['FEAGI_sockets']['general']['burst_beacon'], flags=zmq.SUB)
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
    opu_channel_address = 'tcp://127.0.0.1:23000'
    feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)

    print("Connecting to FEAGI resources...")
    #
    # # Establish the needed zmq connections for FEAGI communications
    # for entry in controller_settings['FEAGI_sockets']['pub']:
    #     publisher = Pub(address=controller_settings['FEAGI_sockets']['pub'][entry])
    #     print("   Building publisher connections for %s" % entry)
    #
    # for entry in controller_settings['FEAGI_sockets']['sub']:
    #     subscriber = Sub(address=controller_settings['FEAGI_sockets']['sub'][entry])
    #     print("   Building subscriber connections for %s" % entry)

    # Listen and route
    print("Starting the routing engine for ", controller_settings['properties']['mode'])
    print("Communication frequency is set once every %f seconds" % controller_settings['timers']['global_timer'])
    while True:
        if controller_settings['properties']['mode'] == 'rpi':
            print("<< Incomplete RPI Code >>")

        elif controller_settings['properties']['mode'] == 'virtual':
            opu_data = feagi_opu_channel.receive()
            print(opu_data)

        elif controller_settings['properties']['mode'] == 'ros':
            # Cycle through all subscribed ROS topics and publish them to the corresponding FEAGI channel
            print("<< Incomplete ROS Code >>")

        # Cycle through all FEAGI messages and publish them on the corresponding ROS channel
            print("<< Incomplete Code >>")

        else:
            print("Error: Chosen controller mode is not supported!")

        # Cycle through all the

        sleep(controller_settings['timers']['global_timer'])
