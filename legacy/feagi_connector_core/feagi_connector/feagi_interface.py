import os
import sys
import json
import socket
import serial.tools.list_ports
import argparse
import platform
import requests
import threading
import traceback
import pkg_resources
from time import sleep
import struct

import feagi_connector
from feagi_connector import retina
from feagi_connector import router
from feagi_connector import actuators
from feagi_connector import pns_gateway as pns
from feagi_connector.version import __version__
from feagi_connector.flag_settings import available_args


def validate_requirements(requirements_file='requirements.txt'):
    """
    Validates that all packages listed in the given requirements file match their installed versions.

    :param requirements_file: The path to the requirements file. Default is 'requirements.txt'.
    :raises SystemExit: If any package does not match the required version or is not installed.
    """
    with open(requirements_file, 'r') as file:
        requirements = file.readlines()

    mismatched_packages = []

    for requirement in requirements:
        try:
            # Parse the requirement line
            req = pkg_resources.Requirement.parse(requirement.strip())

            # Get the installed version of the package
            installed_version = pkg_resources.get_distribution(req.name).version

            # Compare installed version with the required version
            if installed_version not in req:
                mismatched_packages.append((req.name, req.specs, installed_version))

        except pkg_resources.DistributionNotFound:
            print(f"Package {req.name} is not installed.")
            sys.exit(1)
        except Exception as e:
            print(f"Error processing {requirement}: {e}")
            sys.exit(1)

    if mismatched_packages:
        print("The following packages do not match the versions specified in requirements.txt:")
        for name, required, installed in mismatched_packages:
            required_version = ", ".join([f"{op}{ver}" for op, ver in required])
            print(f"- {name}: required {required_version}, installed {installed}")
        sys.exit(1)
    else:
        print("All packages match the versions specified in requirements.txt.")
        print("Validation complete. Proceeding with application...")


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
    print("Controller's version: ", controller_version, "\nagent version: ", __version__)
    return runtime_data["feagi_state"]


def is_FEAGI_reachable(server_host, server_port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((server_host, server_port))
        return True
    except Exception as e:
        return False


def get_flag_list():
    return available_args


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
    return '/v1/burst_engine/stimulation_period'


def feagi_api_burst_counter():
    return '/v1/burst_engine/burst_counter'


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


def convert_new_networking_into_old_networking(feagi_settings):
    back_to_old_json = {
        "feagi_url": None,
        "feagi_dns": None,
        "feagi_host": None,
        "feagi_api_port": None,
        "nrs_url": "https://us-prd-composer.neurorobotics.studio/v1/public/regional/magic/feagi_session?token="

    }
    if 'feagi_url' in feagi_settings:
        ip = feagi_settings['feagi_url'].split('//')
    else:
        ip = ['https://', '127.0.0.1']  # default to localhost
    back_to_old_json['feagi_host'] = ip[1]  # grab ip only
    back_to_old_json['feagi_api_port'] = feagi_settings['feagi_api_port']
    return back_to_old_json


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


def opu_calculator(feagi_data, cortical_id):
    new_processed_data = {}

    # iterate each group of xy
    grouped_data = {}
    for key, value in feagi_data.items():
        xy_key = (key[0], key[1])
        if xy_key not in grouped_data:
            grouped_data[xy_key] = []
        grouped_data[xy_key].append((key[2], value))

    # calculate each group
    for xy_key, points in grouped_data.items():
        add_value = 0.0
        total_points = len(points)
        for z, value in points:
            add_value += (value * (z + 1)) / pns.full_list_dimension[cortical_id]['cortical_dimensions'][2]
        if add_value != 0.0:
            new_processed_data[xy_key] = add_value / total_points

    return new_processed_data


def opu_processor(data):
    try:
        opu_data = data["opu_data"]
        processed_opu_data = {}
        if opu_data is not None:
            for cortical_id in opu_data:
                if cortical_id in pns.full_list_dimension:
                    if cortical_id not in processed_opu_data:
                        processed_opu_data[cortical_id] = {}
                    if not pns.check_actuator_measure(cortical_id):
                        processed_opu_data[cortical_id] = opu_data[cortical_id]
                    else:
                        processed_opu_data[cortical_id] = opu_calculator(opu_data[cortical_id], cortical_id)
            return processed_opu_data
    except Exception as error:
        print("error: ", error)
        traceback.print_exc()
        # pass


def feagi_data_to_bytes(feagi_data):
    result = bytearray()

    for cortical_id, coords in feagi_data.items():
        # Ensure cortical_id is 6 bytes, padded or trimmed
        cid = cortical_id.encode('ascii')[:6].ljust(6, b'\x00')
        result += cid

        # Write length (number of (x, y, z, v) sets)
        length = len(coords)
        result += struct.pack('<H', length)  # uint16, little endian

        # Add each (x, y, z, v)
        for (x, y, z), v in coords.items():
            result += struct.pack('<III f', x, y, z, v)  # 3x uint32, 1x float32

    return result


def bytes_to_feagi_data(data: bytes):
    feagi_data = {}
    offset = 0

    while offset < len(data):
        # Read 6-byte cortical area ID
        cortical_id = data[offset:offset + 6].rstrip(b'\x00').decode('ascii')
        offset += 6

        # Read length (number of (x, y, z, v) sets)
        length = struct.unpack_from('<H', data, offset)[0]
        offset += 2

        coords = {}
        for _ in range(length):
            x, y, z, v = struct.unpack_from('<III f', data, offset)
            coords[(x, y, z)] = v
            offset += 16  # 4 + 4 + 4 + 4 bytes

        feagi_data[cortical_id] = coords

    return feagi_data


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


def connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities, current_version,
                     bind_flag=False):
    print("Connecting to FEAGI resources...")
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    runtime_data["feagi_state"] = feagi_registration(feagi_auth_url=feagi_auth_url,
                                                     feagi_settings=feagi_settings,
                                                     agent_settings=agent_settings,
                                                     capabilities=capabilities,
                                                     controller_version=current_version)
    api_address = runtime_data['feagi_state']["feagi_url"]
    router.global_api_address = api_address
    agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])
    if not pns.env_exists:
        if bind_flag:
            ipu_channel_address = "tcp://*:" + agent_data_port  # This is for godot to work due to
            # bind unable to use the dns.
        else:
            ipu_channel_address = feagi_outbound(feagi_settings['feagi_host'], agent_data_port)

        print("IPU_channel_address=", ipu_channel_address)
        opu_channel_address = feagi_outbound(feagi_settings['feagi_host'],
                                             runtime_data["feagi_state"]['feagi_opu_port'])

        # ip = '172.28.0.2'
        # opu_channel_address = 'tcp://' + str(ip) + ':3000'
        # ipu_channel_address = 'tcp://' + str(ip) + ':3000'
        feagi_ipu_channel = pub_initializer(ipu_channel_address, bind=bind_flag)
        feagi_opu_channel = sub_initializer(opu_address=opu_channel_address)
        router.global_feagi_opu_channel = feagi_opu_channel
        threading.Thread(target=pns.feagi_listener, args=(feagi_opu_channel,), daemon=True).start()
    else:
        feagi_ipu_channel = None
        feagi_opu_channel = None
        websocket_url = feagi_settings['feagi_dns'].replace("https", "wss") + str("/p9053")
        router.websocket_client_initalize('', '', dns=websocket_url)
        threading.Thread(target=router.websocket_recieve, daemon=True).start()

    if 'output' in capabilities:
        if 'servo' in capabilities['output']:
            actuators.start_servos(capabilities)
        if 'motor' in capabilities['output']:
            actuators.start_motors(capabilities)
    return feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel


def build_up_from_mctl(id):
    action_map = {
        (0, 0): "move_left",
        (0, 1): "yaw_left",
        (0, 2): "roll_left",
        (1, 0): "move_up",
        (1, 1): "move_forward",
        (1, 2): "pitch_forward",
        (2, 0): "move_down",
        (2, 1): "move_backward",
        (2, 2): "pitch_back",
        (3, 0): "move_right",
        (3, 1): "yaw_right",
        (3, 2): "roll_right"
    }
    # Get the action from the dictionary, return None if not found
    return action_map.get((id[0] % 4, id[1]))


def configuration_load(path='./'):
    # NEW JSON UPDATE
    try:
        fcap = open(path + 'capabilities.json')
        json_capabilities = json.load(fcap)
        capabilities = json_capabilities['capabilities']
        fcap.close()
    except Exception as error:
        capabilities = {}
        # print("ERROR: ", error)

    try:
        fnet = open(path + 'networking.json')
        configuration = json.load(fnet)
        feagi_settings = configuration["feagi_settings"]
        agent_settings = configuration['agent_settings']
        feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
        feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
        if 'description' in configuration:
            pns.ver = configuration['description']
        fnet.close()
    except Exception as error:
        # print("ERROR: ", error)
        feagi_settings = {}
        agent_settings = {}

    message_to_feagi = {"data": {}}
    return feagi_settings, agent_settings, capabilities, message_to_feagi, configuration
    # END JSON UPDATE


def list_com_ports():
    ports = serial.tools.list_ports.comports()
    com_list = [port.device for port in ports]
    return com_list


def detect_usb_port(data, previous):
    """Automatically detect the USB port based on the operating system."""
    system = platform.system()
    com_ports = list_com_ports()
    if system == 'Darwin':
        counter = 0
        for name_device in com_ports:
            if 'Bluetooth-Incoming-Port' in name_device:
                com_ports.pop(counter)
            counter += 1
    if len(com_ports) == 0:
        print("No device plugged, exiting the program.")
        sys.exit()

    if len(com_ports) >= 1:
        print("Current available list: ", com_ports)
        for id in range(len(com_ports)):
            if previous in com_ports[id]:
                print(id + 1, ". ", com_ports[id], " (USED PREVIOUSLY)")
            else:
                print(id + 1, ". ", com_ports[id])
        while True:
            user_input = input("Please press number to connect: ")
            if user_input.isdigit():  # Check if the input is a digit
                choice = int(user_input)
                if 1 <= choice <= len(com_ports):  # Ensure the choice is within range
                    return com_ports[choice - 1]  # Return the selected COM port
                else:
                    print(f"Invalid choice. Please select a number between 1 and {len(com_ports)}.")
            else:
                print("Invalid input. Please enter a valid number.")


def reading_parameters_to_confirm_communication(new_settings, configuration, path=".", serial_in_use=False):
    # Check if feagi_connector has arg
    parser = argparse.ArgumentParser(description='enable to use magic link')
    parser.add_argument('-magic_link', '--magic_link',
                        help='Use the API key, as the magic link will be removed in a few updates.', required=False)
    parser.add_argument('-api_key', '--api_key',
                        help='To use the API key from NeuroRobotics Studio to connect with your robot.', required=False)
    parser.add_argument('-ip', '--ip', help='to use feagi_ip', required=False)
    parser.add_argument('-port', '--port', help='to use feagi_port', required=False)
    parser.add_argument('-preview', '--preview', help='To enable the preview of vision',
                        required=False)
    parser.add_argument(
        '-usb_address', '--usb_address',
        help=(
            "Specify the USB port for connecting to your embodiment via a USB cable. "
            "This flag is optional and only needs to be set once."
        ),
        required=False
    )
    args = vars(parser.parse_args())
    if serial_in_use:
        # autogenerated inside networking.json
        if not 'usb_port' in configuration['agent_settings'] or not configuration['agent_settings']['usb_port']:
            system = platform.system()
            default_usb_ports = {
                'Windows': 'COM1',
                'Darwin': '/dev/tty.usbserial',
                'Linux': '/dev/ttyUSB0'
            }
            configuration['agent_settings']['usb_port'] = default_usb_ports.get(system, '')
        existing_port_usb_name = configuration['agent_settings']['usb_port']
        configuration['agent_settings']['usb_port'] = detect_usb_port(configuration['agent_settings']['usb_port'],
                                                                      existing_port_usb_name)
        if existing_port_usb_name != configuration['agent_settings']['usb_port'] or args['usb_address']:
            if args['usb_address']:
                configuration['agent_settings']['usb_port'] = args['usb_address']
            with open(path + 'networking.json', 'w') as f:
                json.dump(configuration, f, indent=4)

    if args['preview']:
        if args['preview'].lower() == 'true':
            retina.preview_flag = True
    if 'feagi_dns' in new_settings:
        print(
            "OLD networking.json DETECTED! Please update your networking.json to latest. Next update will be removed that could crash the feagi controller if the old networking.json is not updated!!!")
        feagi_settings = new_settings
    else:
        feagi_settings = convert_new_networking_into_old_networking(new_settings)
    if args['port']:
        feagi_settings['feagi_opu_port'] = args['port']
    else:
        feagi_settings['feagi_opu_port'] = os.environ.get('FEAGI_OPU_PORT', "3000")

    if (args['api_key'] or pns.env_exists) and not args['ip']:
        if args['api_key']:
            configuration['feagi_settings']['feagi_url'] = feagi_settings['nrs_url'] + args['api_key']  # api-key
            with open(pns.env_current_path, 'w') as f:
                f.write(args['api_key'])
        else:
            with open(pns.env_current_path, 'r') as f:
                configuration['feagi_settings']['feagi_url'] = feagi_settings['nrs_url'] + f.read()
        url_response = json.loads(requests.get(configuration['feagi_settings']['feagi_url']).text)
        feagi_settings['feagi_dns'] = url_response['feagi_url']
        feagi_settings['feagi_api_port'] = url_response['feagi_api_port']
        pns.create_env_again()  # just to update the global variables
    elif args['magic_link']:
        print("The magic link is now deprecated. Please use the '--api_key' instead.")
        sys.exit(1)

    elif args['ip']:
        # # FEAGI REACHABLE CHECKER # #
        feagi_flag = False
        print("retrying...")
        print("Waiting on FEAGI...")
        if args['ip']:
            feagi_settings['feagi_host'] = args['ip']
        if 'feagi_url' in configuration['feagi_settings']:
            configuration['feagi_settings']['feagi_url'] = "http://127.0.0.1"
        if 'feagi_dns' in feagi_settings:
            del feagi_settings['feagi_dns']
        if pns.env_exists:
            os.remove(pns.env_current_path)
            pns.create_env_again()  # just to update the global variables
        if 'magic_link' in feagi_settings:
            del feagi_settings['magic_link']
        with open(path + 'networking.json', 'w') as f:
            json.dump(configuration, f, indent=4)
        while not feagi_flag:
            feagi_flag = is_FEAGI_reachable(os.environ.get('FEAGI_HOST_INTERNAL', feagi_settings["feagi_host"]),
                                            int(os.environ.get('FEAGI_OPU_PORT', feagi_settings['feagi_opu_port'])))
            sleep(2)
    return feagi_settings, configuration


def build_up_from_configuration(path="./", serial_in_use=False):
    feagi_settings, agent_settings, capabilities, message_to_feagi, configuration = configuration_load(path)
    default_capabilities = {}  # It will be generated in process_visual_stimuli. See the
    # overwrite manual
    default_capabilities = pns.create_runtime_default_list(default_capabilities, capabilities)

    feagi_settings, configuration = reading_parameters_to_confirm_communication(feagi_settings,
                                                                                configuration,
                                                                                path,
                                                                                serial_in_use)
    return {
        "feagi_settings": feagi_settings,
        "agent_settings": agent_settings,
        "default_capabilities": default_capabilities,
        "message_to_feagi": message_to_feagi,
        "capabilities": capabilities
    }


def map_value(val, min1, max1, min2, max2):
    """ Performs linear transformation to map value from
    range 1 [min1, max1] to a value in range 2 [min2, max2].

    :param val: value (int/float) being mapped
    :param min1: min of range 1
    :param max1: max of range 1
    :param min2: min of range 2
    :param max2: max of range 2
    :return: value mapped from range 1 to range 2
    """
    if val < min1:
        return min2
    if val > max1:
        return max2

    mapped_value = (val - min1) * ((max2 - min2) / (max1 - min1)) + min2

    if max2 >= mapped_value >= min2:
        return mapped_value
