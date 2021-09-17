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
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    @staticmethod
    def validate(payload):
        """
        This function endures the received payload meets a certain expectations
        """
        try:
            print("<< Incomplete TRY Code >>")
            return True
        except:
            print("<< Incomplete EXCEPTION Code >>")
            return False

    def receive(self):
        payload = self.socket.recv_pyobj()

        if self.validate(payload):
            return payload


def pub_tester():
    publisher = Pub('tcp://0.0.0.0:11000')
    for i in range(10):
        publisher.send(message={"A", "Hello!"})
        sleep(1)


def sub_tester():
    subscriber = Sub('tcp://127.0.0.1:30000')
    for _ in range(10):
        message = subscriber.receive()
        print(message)
        sleep(1)


if __name__ == '__main__':

    print("Testing pub-sub")
    # print("Turning on the Subscriber...")
    sub_tester()
    print("Turning on the Publisher...")
    # pub_tester()


    print("Connecting to FEAGI resources...")

    # Establish the needed zmq connections for FEAGI communications
    for entry in controller_settings['FEAGI_sockets']['pub']:
        publisher = Pub(address=controller_settings['FEAGI_sockets']['pub'][entry])
        print("   Building publisher connections for %s" % entry)

    for entry in controller_settings['FEAGI_sockets']['sub']:
        publisher = Sub(address=controller_settings['FEAGI_sockets']['sub'][entry])
        print("   Building subscriber connections for %s" % entry)

    # Listen and route
    print("Starting the routing engine for ", controller_settings['properties']['mode'])
    print("Communication frequency is set once every %f seconds" % controller_settings['timers']['global_timer'])
    while True:
        if controller_settings['properties']['mode'] == 'rpi':
            print("<< Incomplete RPI Code >>")

        elif controller_settings['properties']['mode'] == 'virtual':
            print("<< Incomplete Virtual Code >>")

        elif controller_settings['properties']['mode'] == 'ros':
            # Cycle through all subscribed ROS topics and publish them to the corresponding FEAGI channel
            print("<< Incomplete ROS Code >>")

        # Cycle through all FEAGI messages and publish them on the corresponding ROS channel
            print("<< Incomplete Code >>")

        else:
            print("Error: Chosen controller mode is not supported!")

        # Cycle through all the

        sleep(controller_settings['timers']['global_timer'])
