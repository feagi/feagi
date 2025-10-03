#!/usr/bin/env python
"""
Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

import asyncio
import threading
from collections import deque
from datetime import datetime
from time import sleep
import traceback
import websockets
from urllib.parse import urlparse, parse_qs
from version import __version__
from feagi_connector import pns_gateway as pns
from feagi_connector import sensors as sensors
from feagi_connector import actuators
from feagi_connector import feagi_interface as feagi
import os
import json
import numpy

ws = deque()
ws_operation = deque()
previous_data = ""
servo_status = {}
gyro = {}
current_device = {}
connected_agents = dict()  # Initalize
connected_agents['capabilities'] = {}
connected_agents['device'] = ""
connected_agents['0'] = False  # By default, it is not connected by client's websocket
muse_data = {}
embodiment_id = {'servo_status': {}, 'acceleration': {}, 'gyro': {}, 'sound_level': {}, 'ir': [], 'ultrasonic': {}}
runtime_data = {"cortical_data": {}, "current_burst_id": None,
                "stimulation_period": 0.01, "feagi_state": None,
                "feagi_network": None}
last_data = {'0': ''}
embodiment_name = {}
try:
    fcap = open('device_bluetooth.json')
    json_embodiment = json.load(fcap)
    embodiment_name = json_embodiment
    fcap.close()
except Exception as error:
    embodiment_name = {}


feagi_misc_to_petoi_token_mapping = {
        0: 'gPb',              # Keep as is
        1: 'f',                # Keep as is
        2: 'ktbl',             # be table
        3: 'kpee',             # pee
        4: 'kstr',             # stretch
        5: 'ksit',             # sit
        6: 'krest',            # rest
        7: 'kjmp',             # jump
        8: 'kfiv',             # high five
        9: 'kpd',              # act dead
        10: 'kpu',             # push ups
        11: 'kwkF',            # walk forward
        12: 'kbk',             # walk backward
        13: 'kL',              # walk left
        14: 'kR',              # walk right
        15: 'kup',                # Removed other irrelevant mappings
        16: 'gPB'
}





def embodiment_id_map(name):
    return embodiment_name[name]


async def bridge_to_godot():
    while True:
        if ws:
            try:
                if ws_operation:
                    if len(ws) > 0:
                        if len(ws) > 2:
                            stored_value = ws.pop()
                            ws.clear()
                            ws.append(stored_value)
                    await ws_operation[0].send(str(ws[0]))
                    ws.pop()
                if "stimulation_period" in runtime_data:
                    sleep(runtime_data["stimulation_period"])
            except Exception as error:
                print("error: ", error)
                sleep(0.001)
        else:
            sleep(0.001)


def feagi_to_petoi_id(device_id):
    mapping = {
        0: 0,
        1: 8,
        2: 12,
        3: 9,
        4: 13,
        5: 11,
        6: 15,
        7: 10,
        8: 14
    }
    return mapping.get(device_id, None)


def petoi_listen(message, full_data):
    try:
        split_data = message['data'].split()
        received_data = {}
        if len(split_data) == 9:
            for servo_id in range(len(split_data)):
                received_data[str(servo_id)] = int(float(split_data[servo_id]))
            embodiment_id['servo_status'] = received_data
        if len(split_data) == 6:
            embodiment_id['gyro']['0'] = [float(split_data[0]), float(split_data[1]), float(split_data[2])]
            embodiment_id['acceleration']['0'] = [int(split_data[3]), int(split_data[4]), int(split_data[5])]
        else:
            full_data = message
    except Exception as Error_case:
        pass
        print("error: ", Error_case)
        traceback.print_exc()
    return full_data


def microbit_listen(message):
    ir_list = []
    if message[0] == 'f':
        pass
    else:
        ir_list.append(0)
    if message[1] == 'f':
        pass
    else:
        ir_list.append(1)
    try:
        x_acc = int(message[2:6])
        y_acc = int(message[6:10])
        z_acc = int(message[10:14])
        ultrasonic = float(message[14:16])
        sound_level = int(message[16:18])
        # Store values in dictionary
        embodiment_id['ir'] = ir_list
        embodiment_id['ultrasonic'] = ultrasonic / 25
        embodiment_id['acceleration']['0'] = [x_acc,  y_acc, z_acc]
        embodiment_id['sound_level'] = {sound_level}
        return
    except Exception as Error_case:
        pass
        # print("error: ", Error_case)
        # print("raw: ", message)


def bridge_operation():
    asyncio.run(bridge_to_godot())


async def echo(websocket, path):
    """
    The function echoes the data it receives from other connected websockets
    and sends the data from FEAGI to the connected websockets.
    """
    try:
        current_device['name'] = []
        full_data = ''
        connected_agents['0'] = True  # Since this section gets data from client, its marked as true
        query_params = parse_qs(urlparse(path).query)
        device = query_params.get('device', [None])[0]  # Default to None if 'device' is not provided
        try:
            current_device['name'] = [embodiment_id_map(device)]
        except:
            current_device['name'] = [device]
        if not ws_operation:
            ws_operation.append(websocket)
        else:
            ws_operation[0] = websocket
        async for message in websocket:
            data_from_bluetooth = json.loads(message)
            for device_name in data_from_bluetooth:
                name_of_device = device_name
                if device_name == 'capabilities':
                    connected_agents['capabilities'] = data_from_bluetooth['capabilities']
                    if 'petoi' in current_device['name'] and connected_agents['capabilities']:
                        feagi_servo_data_to_send = 'i '
                        for position in connected_agents['capabilities']['output']['servo']:
                            feagi_servo_data_to_send += str(feagi_to_petoi_id(int(position))) + " " + str(
                                connected_agents['capabilities']['output']['servo'][position]['default_value']) + " "
                        actuators.start_servos(connected_agents['capabilities'])
                        ws.append(feagi_servo_data_to_send)
                    break
                if "em-" in device_name:
                    name_of_device = embodiment_id_map(name_of_device)
                if name_of_device not in current_device['name']:
                    if name_of_device == 'petoi' and connected_agents['capabilities']:
                        feagi_servo_data_to_send = 'i '
                        for position in connected_agents['capabilities']['output']['servo']:
                            feagi_servo_data_to_send += str(feagi_to_petoi_id(int(position))) + " " + str(connected_agents['capabilities']['output']['servo'][position]['default_value']) + " "
                        actuators.start_servos(connected_agents['capabilities'])
                        ws.append(feagi_servo_data_to_send)
                    elif name_of_device in ['microbit']:
                        pass
                    else:
                        break
                    current_device['name'].append(name_of_device)
                connected_agents['0'] = True  # Since this section gets data from client, its marked as true

                if not ws_operation:
                    ws_operation.append(websocket)
                else:
                    ws_operation[0] = websocket

                if name_of_device == "microbit":
                    microbit_listen(data_from_bluetooth[device_name]['data'])
                elif name_of_device == "petoi":
                    full_data = petoi_listen(data_from_bluetooth[device_name], full_data)  # Needs to add
                elif name_of_device == "muse":
                    muse_listen(data_from_bluetooth[device_name])
                elif name_of_device == "generic":
                    print("generic")
                    pass  # Needs to figure how to address this
                else:
                    print("unknown device")
                    print("message: ", data_from_bluetooth, " and device: ", current_device['name'])
    except Exception as error:
        print("error: ", error)
        traceback.print_exc()
    connected_agents['0'] = False  # Once client disconnects, mark it as false
    muse_data.clear()
    current_device['name'].clear()
    for i in embodiment_id:
        if isinstance(embodiment_id[i], dict):
            embodiment_id[i].clear()
        elif isinstance(embodiment_id[i], list):
            embodiment_id[i].clear()
        else:
            embodiment_id[i] = None


async def main():
    """
    The main function handles the websocket and spins the asyncio to run the echo function
    infinitely until it exits. Once it exits, the function will resume to the next new websocket.
    """
    async with websockets.serve(echo, agent_settings["godot_websocket_ip"],
                                agent_settings['godot_websocket_port']):
        await asyncio.Future()  # run forever


def websocket_operation():
    """
    WebSocket initialized to call the echo function using asyncio.
    """
    asyncio.run(main())


def muse_listen(obtained_data):
    dict_from_muse = obtained_data
    if dict_from_muse['type'] not in muse_data:
        muse_data[dict_from_muse['type']] = {}
    if dict_from_muse['type'] == 'eeg':
        muse_data['eeg'][dict_from_muse['data']['electrode']] = dict_from_muse['data']['samples']
    if dict_from_muse['type'] == 'acceleration':
        for i in range(len(dict_from_muse['data']['samples'])):
            muse_data['acceleration'][i] = dict_from_muse['data']['samples'][i]
    if dict_from_muse['type'] == 'telemetry':
        muse_data['telemetry']['battery'] = dict_from_muse['data']['batteryLevel']


def petoi_action(obtained_data):
    WS_STRING = ""
    servo_data = actuators.get_servo_data(obtained_data)
    recieve_servo_position_data = actuators.get_servo_position_data(obtained_data)
    recieved_misc_data = actuators.get_generic_opu_data_from_feagi(obtained_data, 'misc')

    if recieve_servo_position_data:
        servo_for_feagi = 'i '
        for device_id in recieve_servo_position_data:
            new_power = int(recieve_servo_position_data[device_id])
            servo_for_feagi += str(feagi_to_petoi_id(device_id)) + " " + str(new_power) + " "
        WS_STRING = servo_for_feagi

    if recieved_misc_data:
        # Note: Only the last command is being considered and the rest are disposed
        for data_point in recieved_misc_data:
            # new_data = feagi_misc_to_petoi_token_mapping.get(data_point)
            # if new_data != last_data['0']: # Add this to reduce the chance to crash petoi
            WS_STRING = feagi_misc_to_petoi_token_mapping.get(data_point)
                # last_data['0'] = new_data
    if servo_data:
        servo_for_feagi = 'i '
        for device_id in servo_data:
            power = int(servo_data[device_id])
            servo_for_feagi += str(feagi_to_petoi_id(device_id)) + " " + str(power) + " "
        WS_STRING = servo_for_feagi
    if WS_STRING != "":
        ws.append(WS_STRING)


def microbit_action(obtained_data):
    recieve_motor_data = actuators.get_motor_data(obtained_data)
    WS_STRING = ""

    if recieve_motor_data:
        updated_motor=0
        for motor_id in [0, 1]:
            data_power = 0
            if motor_id in recieve_motor_data:
                data_power = recieve_motor_data[motor_id]
                if data_power == 100:
                    data_power -= 1
                elif data_power == -100:
                    data_power += 1

            if motor_id == 0:
                if data_power < 0:
                    updated_motor = 1
            elif motor_id == 1:
                if data_power < 0:
                    updated_motor = 3
                else:
                    updated_motor = 2
            WS_STRING += str(updated_motor) + str(abs(data_power)).zfill(2)

    if WS_STRING == "000200":
        WS_STRING = ""  # so we don't spam microbit with no power

    if WS_STRING != "":
        WS_STRING = WS_STRING + "#"
        ws.append(WS_STRING)


def feagi_main(feagi_auth_url, feagi_settings, agent_settings, capabilities, message_to_feagi):
    global runtime_data # Literally no reason for it to be here. Somehow it is needed?????
    feagi_flag = False
    print("Waiting on FEAGI...")
    while not feagi_flag:
        feagi_flag = feagi.is_FEAGI_reachable(
            os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
            int(os.environ.get('FEAGI_OPU_PORT', "3000"))
        )
        sleep(0.1)

    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    print("FEAGI AUTH URL ------- ", feagi_auth_url)

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, connected_agents['capabilities'],
                               __version__)
    threading.Thread(target=pns.feagi_listener, args=(feagi_opu_channel,), daemon=True).start()
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    runtime_data['acceleration'] = {}

    actuators.start_motors(connected_agents['capabilities'])  # initialize motors for you.
    while connected_agents['0']:
        try:
            message_from_feagi = pns.message_from_feagi
            # OPU section STARTS
            if message_from_feagi:
                pns.check_genome_status_no_vision(message_from_feagi)
                feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi, feagi_settings['feagi_burst_speed'])
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                if 'name' in current_device:
                    if "microbit" in current_device['name']:
                        microbit_action(obtained_signals)
                    elif "petoi" in current_device['name']:
                        petoi_action(obtained_signals)

            # OPU section ENDS
            if embodiment_id['ultrasonic']:
                message_to_feagi = sensors.create_data_for_feagi(sensor='proximity', capabilities=connected_agents['capabilities'], message_to_feagi=message_to_feagi,
                                                                 current_data=embodiment_id['ultrasonic'], measure_enable=True)

            if embodiment_id['acceleration']:
                if pns.full_template_information_corticals:
                    if 'infrared' in connected_agents['capabilities']['input']:
                        message_to_feagi = sensors.convert_ir_to_ipu_data(embodiment_id['ir'], len(connected_agents['capabilities']['input']['infrared']), message_to_feagi)
                    # The IR will need to turn the inverse IR on if it doesn't detect. This would confuse humans when
                    # cutebot is not on. So the solution is to put this under the acceleration. It is under acceleration
                    # because without acceleration, the micro:bit is not on. This leverages the advantage to detect if it
                    # is still on.
                    message_to_feagi = sensors.create_data_for_feagi(sensor='accelerometer', capabilities=connected_agents['capabilities'], message_to_feagi=message_to_feagi,
                                                                     current_data=embodiment_id['acceleration'], symmetric=True,
                                                                     measure_enable=True)
            if embodiment_id['servo_status']:
                message_to_feagi = sensors.create_data_for_feagi('servo_position',
                                                                 connected_agents['capabilities'],
                                                                 message_to_feagi,
                                                                 current_data=embodiment_id['servo_status'],
                                                                 symmetric=True)
            if embodiment_id['gyro']:
                # print("gyro: ", petoi_data['gyro'])
                message_to_feagi = sensors.create_data_for_feagi(
                    sensor='gyro',
                    capabilities=connected_agents['capabilities'],
                    message_to_feagi=message_to_feagi,
                    current_data=embodiment_id['gyro'],
                    symmetric=True,
                    measure_enable=True)
            if embodiment_id['acceleration']:
                # print("acc: ", petoi_data['acceleration'])
                message_to_feagi = sensors.create_data_for_feagi(
                    sensor='accelerometer',
                    capabilities=connected_agents['capabilities'],
                    message_to_feagi=message_to_feagi,
                    current_data=embodiment_id['acceleration'],
                    symmetric=True,
                    measure_enable=True)

            message_to_feagi['timestamp'] = datetime.now()
            message_to_feagi['counter'] = msg_counter
            message_to_feagi = sensors.add_agent_status(connected_agents['0'],
                                                        message_to_feagi,
                                                        agent_settings)
            sleep(feagi_settings['feagi_burst_speed'])
            pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
            message_to_feagi.clear()
        except Exception as e:
            print("ERROR: ", e)
            traceback.print_exc()
            break


if __name__ == '__main__':
    # NEW JSON UPDATE
    configuration = feagi.build_up_from_configuration()
    feagi_settings = configuration["feagi_settings"]
    agent_settings = configuration['agent_settings']
    capabilities = configuration['capabilities']
    feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
    feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
    agent_settings['godot_websocket_port'] = os.environ.get('WS_MICROBIT_PORT', "9052")
    message_to_feagi = {}
    # END JSON UPDATE
    threading.Thread(target=websocket_operation, daemon=True).start()
    # threading.Thread(target=bridge_to_godot, daemon=True).start()
    threading.Thread(target=bridge_operation, daemon=True).start()
    print("Waiting on a device to connect....")
    while not connected_agents['capabilities']:
        sleep(0.5)
    while True:
        while not connected_agents['capabilities']:
            sleep(0.1) # Repeated but inside loop
        if connected_agents['capabilities']:
            capabilities = connected_agents['capabilities']
            feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
            feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
            agent_settings['godot_websocket_port'] = os.environ.get('WS_GODOT_GENERIC_PORT', "9055")
        feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
        print("FEAGI AUTH URL ------- ", feagi_auth_url)
        try:
            feagi_main(feagi_auth_url, feagi_settings, agent_settings, capabilities,
                       message_to_feagi)
        except Exception as e:
            print(f"Controller run failed", e)
            traceback.print_exc()
            sleep(2)
        connected_agents['device'] = ""
        connected_agents['capabilities'] = {}