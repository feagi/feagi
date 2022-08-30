import configuration
import zmq
import socket
import requests
from time import sleep


def app_host_info():
    host_name = socket.gethostname()
    ip_address = socket.gethostbyname(socket.gethostname())
    return {"ip_address": ip_address, "host_name": host_name}


class Pub:
    def __init__(self, address):
        context = zmq.Context()
        self.address = address
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)
        print("Sent:\n", message, "... from ", self.address, "\n\n")


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
            # todo: define validation criteria
            # print("<< Incomplete TRY Code >>")
            return True
        except Exception:
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


def register_with_feagi(app_name, feagi_host, api_port, app_capabilities, host_info):
    """
    To trade information between FEAGI and Controller

    Controller                      <--     FEAGI(IPU/OPU socket info)
    Controller (Capabilities)       -->     FEAGI
    """

    api_address = 'http://' + feagi_host + ':' + api_port

    registration_endpoint = '/v1/feagi/register'
    network_endpoint = '/v1/feagi/feagi/network'
    stimulation_period_endpoint = '/v1/feagi/feagi/burst_engine/stimulation_period'
    burst_counter_endpoint = '/v1/feagi/feagi/burst_engine/burst_counter'

    registration_data = {"source": app_name,
                         "host": host_info["ip_address"],
                         "capabilities": app_capabilities}

    registration_complete = False
    app_port_id = 'feagi_inbound_port_' + app_name
    feagi_settings = requests.get(api_address + network_endpoint).json()

    while not registration_complete:

        print("Registration data:", registration_data)

        feagi_registration_result = requests.post(api_address + registration_endpoint, data=registration_data)

        print("FEAGI registration results: ", feagi_registration_result)

        zmq_address = 'tcp://' + feagi_host + ':' + feagi_settings[app_port_id]

        print('Awaiting connection with FEAGI at...', zmq_address)
        # subscriber = Sub(address=zmq_address, flags=zmq.SUB)

        # Receive FEAGI settings
        feagi_settings['burst_duration'] = requests.get(api_address + stimulation_period_endpoint).json()
        feagi_settings['burst_counter'] = requests.get(api_address + burst_counter_endpoint).json()

        print("\nFEAGI settings received as:\n", feagi_settings, "\n\n")
        if feagi_settings and feagi_settings['burst_duration'] and feagi_settings['burst_counter']:
            print("\n\n\n\nRegistration is complete....")
            registration_complete = True
        sleep(1)

    # Transmit Controller Capabilities
    pub_address = "tcp://0.0.0.0:" + feagi_settings[app_port_id]
    publisher = Pub(address=pub_address)
    publisher.send(app_capabilities)

    return feagi_settings


def feagi_registration(feagi_host, api_port, host_info):
    runtime_data = {
        "host_network": {},
        "feagi_state": None
    }
    runtime_data["host_network"]["host_name"] = host_info["host_name"]
    runtime_data["host_network"]["ip_address"] = host_info["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("Awaiting registration with FEAGI...")
        try:
            runtime_data["feagi_state"] = register_with_feagi(app_name=configuration.app_name,
                                                              feagi_host=feagi_host,
                                                              api_port=api_port,
                                                              app_capabilities=configuration.capabilities,
                                                              host_info=runtime_data["host_network"]
                                                              )
        except Exception:
            pass
        sleep(1)
    return runtime_data["feagi_state"]


def block_to_array(block_ref):
    block_id_str = block_ref.split('-')
    array = [int(x) for x in block_id_str]
    return array


def feagi_setting_for_registration():
    """
    Generate all needed information and return the full data to make it easier to connect with
    FEAGI
    """
    feagi_ip_host = configuration.network_settings["feagi_host"]
    api_data = configuration.network_settings["feagi_api_port"]
    return feagi_ip_host, api_data


def feagi_gui_address(feagi_ip_host, api_data):
    """
    return a full path to api
    """
    return 'http://' + feagi_ip_host + ':' + api_data


def feagi_api_burst_engine():
    return '/v1/feagi/feagi/burst_engine/stimulation_period'


def feagi_api_burst_counter():
    return '/v1/feagi/feagi/burst_engine/burst_counter'


def feagi_inbound(feagi_inbound_port):
    """
    Return the zmq address of inbound
    """
    return 'tcp://0.0.0.0:' + feagi_inbound_port


def feagi_outbound(feagi_ip_host, feagi_outbound_port):
    """
    Return the zmq address of outbound
    """
    return 'tcp://' + feagi_ip_host + ':' + \
           feagi_outbound_port


def msg_processor(self, msg, msg_type):
    # TODO: give each subclass a specific msg processor method?
    # TODO: add an attribute that explicitly defines message type (instead of parsing topic name)?
    if 'ultrasonic' in msg_type and msg.ranges[1]:
        return {
            msg_type: {
                idx: val for idx, val in enumerate([msg.ranges[1]])
            }
        }
    elif 'IR' in msg_type:
        rgb_vals = list(msg.data)
        avg_intensity = sum(rgb_vals) // len(rgb_vals)

        sensor_topic = msg_type.split('/')[0]
        sensor_id = int(''.join(filter(str.isdigit, sensor_topic)))

        # print("\n***\nAverage Intensity = ", avg_intensity)
        if avg_intensity > configuration.capabilities["infrared"]["threshold"]:
            return {
                'ir': {
                    sensor_id: False
                }
            }
        else:
            return {
                'ir': {
                    sensor_id: True
                }
            }


def compose_message_to_feagi(original_message, data, battery):
    """
    accumulates multiple messages in a data structure that can be sent to feagi
    """
    runtime_data = dict()
    runtime_data["battery_charge_level"] = battery
    message_to_feagi = data
    if "data" not in message_to_feagi:
        message_to_feagi["data"] = dict()
    if "sensory_data" not in message_to_feagi["data"]:
        message_to_feagi["data"]["sensory_data"] = dict()
    if original_message is not None:
        for sensor in original_message:
            if sensor not in message_to_feagi["data"]["sensory_data"]:
                message_to_feagi["data"]["sensory_data"][sensor] = dict()
            for sensor_data in original_message[sensor]:
                if sensor_data not in message_to_feagi["data"]["sensory_data"][sensor]:
                    message_to_feagi["data"]["sensory_data"][sensor][sensor_data] = original_message[sensor][
                        sensor_data]
        message_to_feagi["data"]["sensory_data"]["battery"] = {1: runtime_data["battery_charge_level"] / 100}
    return message_to_feagi, runtime_data["battery_charge_level"]


def opu_processor(data):
    try:
        processed_opu_data = {'motor': {}, 'servo': {}, 'battery': {}, 'discharged_battery': {}, 'reset': {},
                              'camera': {}}
        opu_data = data["opu_data"]
        if opu_data is not None:
            if 'o__mot' in opu_data:
                for data_point in opu_data['o__mot']:
                    data_point = block_to_array(data_point)
                    device_id = data_point[0]
                    device_power = data_point[2]
                    processed_opu_data['motor'][device_id] = device_power

            if 'o__ser' in opu_data:
                if opu_data['o__ser']:
                    for data_point in opu_data['o__ser']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        processed_opu_data['servo'][device_id] = device_power

            if 'o_cbat' in opu_data:
                if opu_data['o__bat']:
                    for data_point in opu_data['o_cbat']:
                        intensity = data_point[2]
                        processed_opu_data['battery'] = intensity

            if 'o_dbat' in opu_data:
                if opu_data['o__bat']:
                    for data_point in opu_data['o_dbat']:
                        intensity = data_point
                        processed_opu_data['battery'] = intensity

            if 'o_init' in opu_data:
                if opu_data['o_init']:
                    for data_point in opu_data['o_init']:
                        position_index = data_point
                        processed_opu_data['reset'] = position_index

            return processed_opu_data
    except Exception:
        # print("error: ", e)
        pass


def control_data_processor(data):
    control_data = data['control_data']
    if control_data is not None:
        if 'motor_power_coefficient' in control_data:
            configuration.capabilities["motor"]["power_coefficient"] = float(control_data['motor_power_coefficient'])
        if 'robot_starting_position' in control_data:
            for position_index in control_data['robot_starting_position']:
                configuration.capabilities["position"][position_index]["x"] = \
                    float(control_data['robot_starting_position'][position_index][0])
                configuration.capabilities["position"][position_index]["y"] = \
                    float(control_data['robot_starting_position'][position_index][1])
                configuration.capabilities["position"][position_index]["z"] = \
                    float(control_data['robot_starting_position'][position_index][2])
        return configuration.capabilities["motor"]["power_coefficient"], \
               configuration.capabilities["position"]


def model_data_processor(data, full_model_data):
    model_data = data['model_data']
    current_data = full_model_data
    update_flag = False
    if model_data is not None:
        if 'gazebo_floor_img_file' in model_data:
            if current_data["floor_img"] != model_data['gazebo_floor_img_file']:
                current_data['floor_img'] = model_data['gazebo_floor_img_file']
                return current_data, "floor"
        if 'robot_sdf_file_name' in model_data:
            if current_data["robot_model"] != model_data['robot_sdf_file_name']:
                current_data['robot_model'] = model_data['robot_sdf_file_name']
                current_data['robot_model_path'] = model_data['robot_sdf_file_name_path']
                return current_data, "robot"
        if 'mu' in model_data:
            if current_data["mu"] != model_data['mu']:
                current_data["mu"] = float(model_data['mu'])
                update_flag = True
        if 'mu2' in model_data:
            if current_data["mu2"] != model_data["mu2"]:
                current_data["mu2"] = float(model_data['mu2'])
                update_flag = True
        if 'fdir' in model_data:
            if current_data["fdir1"] != model_data['fdir']:
                current_data["fdir1"] = model_data['fdir']
                update_flag = True
        if 'slip1' in model_data:
            if current_data["slip1"] != model_data['slip1']:
                current_data["slip1"] = float(model_data['slip1'])
                update_flag = True
        if 'slip2' in model_data:
            if current_data["slip2"] != model_data['slip2']:
                current_data["slip2"] = float(model_data['slip2'])
                update_flag = True
        if update_flag:
            return current_data, update_flag
