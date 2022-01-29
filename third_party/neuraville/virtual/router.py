"""

"""

import zmq


class Pub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)
        print("Sent:\n", message, "\n\n")


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


def handshake_with_feagi(address, capabilities):
    """
    To trade information between FEAGI and Controller

    Controller                      <--     FEAGI(IPU/OPU socket info)
    Controller (Capabilities)       -->     FEAGI
    """

    print('Awaiting connection with FEAGI at...', address)
    subscriber = Sub(address=address, flags=zmq.SUB)

    # Receive FEAGI settings
    feagi_settings = subscriber.receive()
    print("Connection to FEAGI has been established")
    print("\nFEAGI settings received as:\n", feagi_settings, "\n\n")

    # Transmit Controller Capabilities
    pub_address = "tcp://0.0.0.0:" + feagi_settings['sockets']['feagi_inbound_port']
    publisher = Pub(address=pub_address)
    publisher.send(capabilities)

    return feagi_settings


# def register_with_feagi():
#     """
#     Provides FEAGI the IP address for the ZMQ IPU channel that the sensory data
#     """
#     print("Registering router with FEAGI")
#     publisher_ = Pub('tcp://0.0.0.0:11000')
#
#     # todo: need to send a set of capabilities to FEAGI
#     publisher_.send(message={"A", "Hello!"})
#
#     print("Router registration has successfully completed!")
