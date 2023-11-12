import traceback
from feagi_agent import router
from feagi_agent.version import __version__
from time import sleep
import requests
import socket


def pub_initializer(ipu_address, bind=True):
    return router.Pub(address=ipu_address, bind=bind)


def sub_initializer(opu_address, flags=router.zmq.NOBLOCK):
    return router.Sub(address=opu_address, flags=flags)


def feagi_registration(feagi_auth_url, feagi_settings, agent_settings, capabilities,
                       controller_version):
    host_info = router.app_host_info()
    runtime_data = {
        "host_network": {},
        "feagi_state": None
    }
    runtime_data["host_network"]["host_name"] = host_info["host_name"]
    runtime_data["host_network"]["ip_address"] = host_info["ip_address"]
    agent_settings['agent_ip'] = host_info["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("\nAwaiting registration with FEAGI...")
        try:
            runtime_data["feagi_state"] = \
                router.register_with_feagi(feagi_auth_url, feagi_settings, agent_settings,
                                           capabilities, controller_version, __version__)
        except Exception as e:
            print("ERROR__: ", e, traceback.print_exc())
            pass
        sleep(1)
    print("\nversion: ", controller_version, "\n")
    print("\nagent version: ", __version__, "\n")
    return runtime_data["feagi_state"]


def block_to_array(block_ref):
    block_id_str = block_ref.split('-')
    array = [int(x) for x in block_id_str]
    return array


def is_FEAGI_reachable(server_host, server_port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((server_host, server_port))
        return True
    except Exception as e:
        return False


def feagi_setting_for_registration(feagi_settings, agent_settings):
    """
    Generate all needed information and return the full data to make it easier to connect with
    FEAGI
    """
    feagi_ip_host = feagi_settings["feagi_host"]
    api_port = feagi_settings["feagi_api_port"]
    app_data_port = agent_settings["agent_data_port"]
    return feagi_ip_host, api_port, app_data_port


def feagi_api_burst_engine():
    return '/v1/feagi/feagi/burst_engine/stimulation_period'


def feagi_api_burst_counter():
    return '/v1/feagi/feagi/burst_engine/burst_counter'


def feagi_inbound(feagi_inbound_port):
    """
    Return the zmq address of inbound
    """
    return 'tcp://0.0.0.0:' + feagi_inbound_port


def feagi_outbound(feagi_ip_host, feagi_opu_port):
    """
    Return the zmq address of outbound
    """
    return 'tcp://' + feagi_ip_host + ':' + \
           feagi_opu_port


def msg_processor(self, msg, msg_type, capabilities):
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
        if avg_intensity > capabilities["infrared"]["threshold"]:
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


def compose_message_to_feagi(original_message, data=None, battery=0):
    """
    accumulates multiple messages in a data structure that can be sent to feagi
    """
    if data is None:
        data = {}
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
                    message_to_feagi["data"]["sensory_data"][sensor][sensor_data] = \
                        original_message[sensor][
                            sensor_data]
        message_to_feagi["data"]["sensory_data"]["battery"] = {
            1: runtime_data["battery_charge_level"] / 100}
    return message_to_feagi, runtime_data["battery_charge_level"]


def opu_processor(data):
    try:
        processed_opu_data = {'motor': {}, 'servo': {}, 'battery': {}, 'discharged_battery': {},
                              'reset': {}, 'camera': {}, 'misc': {}, 'navigation': {}, 'speed': {},
                              'servo_position': {}, "led": {}, "vision_resolution": {},
                              "vision_acuity": {}}
        opu_data = data["opu_data"]
        if opu_data is not None:
            if 'o__mot' in opu_data:
                for data_point in opu_data['o__mot']:
                    processed_data_point = block_to_array(data_point)
                    device_id = processed_data_point[0]
                    device_power = opu_data['o__mot'][data_point]
                    processed_opu_data['motor'][device_id] = device_power
            if 'o__ser' in opu_data:
                if opu_data['o__ser']:
                    for data_point in opu_data['o__ser']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['o__ser'][data_point]
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
            if 'o_misc' in opu_data:
                if opu_data['o_misc']:
                    for data_point in opu_data['o_misc']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['o_misc'][data_point]
                        processed_opu_data['misc'][device_id] = device_power
            if 'o__led' in opu_data:
                if opu_data['o__led']:
                    for data_point in opu_data['o__led']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = opu_data['o__led'][data_point]
                        processed_opu_data['led'][device_id] = device_power
            if 'o__nav' in opu_data:
                if opu_data['o__nav']:
                    for data_point in opu_data['o__nav']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        device_power = device_power - 10
                        processed_opu_data['navigation'][device_id] = device_power
            if 'o__spd' in opu_data:
                if opu_data['o__spd']:
                    for data_point in opu_data['o__spd']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        processed_opu_data['speed'][device_id] = device_power
            if 'o__pos' in opu_data:
                if opu_data['o__pos']:
                    for data_point in opu_data['o__pos']:
                        processed_data_point = block_to_array(data_point)
                        device_id = processed_data_point[0]
                        device_power = processed_data_point[2]
                        processed_opu_data['servo_position'][device_id] = device_power
            if 'o_vres' in opu_data:
                if opu_data['o_vres']:
                    for data_point in opu_data['o_vres']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        processed_opu_data['vision_resolution'][device_id] = device_power
            if 'o_vact' in opu_data:
                if opu_data['o_vact']:
                    for data_point in opu_data['o_vact']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        processed_opu_data['vision_acuity'][device_id] = device_power
            return processed_opu_data
    except Exception as error:
        print("error: ", error)
        traceback.print_exc()
        # pass


def control_data_processor(data):
    control_data = data['control_data']
    if control_data is not None:
        if 'motor_power_coefficient' in control_data:
            configuration.capabilities["motor"]["power_coefficient"] = float(
                control_data['motor_power_coefficient'])
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


def connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities, current_version):
    print("Connecting to FEAGI resources...")
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    runtime_data["feagi_state"] = feagi_registration(feagi_auth_url=feagi_auth_url,
                                                     feagi_settings=feagi_settings,
                                                     agent_settings=agent_settings,
                                                     capabilities=capabilities,
                                                     controller_version=current_version)
    api_address = runtime_data['feagi_state']["feagi_url"]
    agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])
    ipu_channel_address = feagi_outbound(feagi_settings['feagi_host'], agent_data_port)

    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = feagi_outbound(feagi_settings['feagi_host'],
                                         runtime_data["feagi_state"]['feagi_opu_port'])

    feagi_ipu_channel = pub_initializer(ipu_channel_address, bind=False)
    feagi_opu_channel = sub_initializer(opu_address=opu_channel_address)
    return feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel
