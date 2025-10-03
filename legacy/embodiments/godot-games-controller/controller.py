#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
import os
import json
import zlib
import asyncio
import traceback
import threading
import websockets
import numpy as np
from time import sleep
from collections import deque
from version import __version__
from feagi_connector import sensors
from feagi_connector import actuators
from feagi_connector import retina as retina
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as feagi

ws = deque()
zmq_queue = deque()
webcam_size = {'size': []}
runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": 0.01,
                "feagi_state": None, "feagi_network": None, 'accelerator': {}}
camera_data = {"vision": None}
connected_agents = dict()  # Initialize
connected_agents['0'] = False  # By default, it is not connected by client's websocket
connected_agents['capabilities'] = {}
connected_agents['device'] = ""
feagi.validate_requirements('requirements.txt')  # you should get it from the boilerplate generator


async def echo(websocket):
    """
    The function echoes the data it receives from other connected websockets
    and sends the data from FEAGI to the connected websockets.
    """
    try:
        async for message in websocket:
            connected_agents['0'] = True  # Since this section gets data from client, its marked as true
            if not ws_operation:
                ws_operation.append(websocket)
            else:
                ws_operation[0] = websocket
            if message != b'{}':
                zmq_queue.append(message)
    except Exception as error:
        if "stimulation_period" in runtime_data:
            sleep(runtime_data["stimulation_period"])
        pass
        # print("ERROR!: ", error)
        # traceback.print_exc()
    connected_agents['0'] = False  # Once client disconnects, mark it as false
    camera_data['vision'] = None


def godot_to_feagi():
    while True:
        if len(zmq_queue) > 0:
            if len(zmq_queue) > 2:
                stored_value = zmq_queue.pop()
                zmq_queue.clear()
                zmq_queue.append(stored_value)
            message = zmq_queue[0]
            obtain_list = zlib.decompress(message)
            try:
                # First, try loading the decompressed message as JSON
                new_data = json.loads(
                    obtain_list.decode('utf-8'))  # only ot check if its list or not. list cant do this
                new_data = json.loads(obtain_list)
                if 'capabilities' in new_data:
                    connected_agents['capabilities'] = new_data['capabilities']
                if 'gyro' in new_data:
                    gyro['gyro'] = new_data['gyro']
                if 'camera' in new_data:
                    if connected_agents['capabilities']:
                        for device_id in new_data['camera']:
                            string_array = new_data['camera'][device_id]
                            new_cam = np.array(string_array).astype(np.uint8)
                            image = new_cam.reshape(
                                connected_agents['capabilities']['input']['camera'][device_id]['camera_resolution'][0],
                                connected_agents['capabilities']['input']['camera'][device_id]['camera_resolution'][1],
                                3)
                            if camera_data['vision'] is None:
                                camera_data['vision'] = dict()
                            camera_data['vision'][device_id] = image
                if 'acceleration' in new_data:
                    acc['accelerator'] = new_data['acceleration']
                if 'proximity' in new_data:
                    prox['proximity'] = new_data['proximity']
            except UnicodeDecodeError as decode_error:
                # If not JSON, assume it's a list
                try:
                    string_array = list(obtain_list)
                    new_depth = (string_array[0] << 8) | string_array[1]
                    new_width = (string_array[2] << 8) | string_array[3]
                    image = string_array[4:]  # This removes the first two elements
                    raw_frame = retina.RGB_list_to_ndarray(image, [new_width, new_depth])
                    raw_frame = retina.update_astype(raw_frame)
                    camera_data['vision'] = raw_frame
                except Exception as list_error:
                    print(f"Error processing list: {list_error}")
                    # traceback.print_exc()
            zmq_queue.pop()
        if "stimulation_period" in runtime_data:
            sleep(runtime_data["stimulation_period"])


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
                    json_data = json.dumps(ws[0])
                    await ws_operation[0].send(json_data)
                    ws.pop()
                if "stimulation_period" in runtime_data:
                    sleep(runtime_data["stimulation_period"])
            except Exception as error:
                # print("error: ", error)
                sleep(0.001)
        else:
            sleep(0.001)


def bridge_operation():
    asyncio.run(bridge_to_godot())


async def main():
    """
    The main function handles the websocket and spins the asyncio to run the echo function
    infinitely until it exits. Once it exits, the function will resume to the next new websocket.
    """
    async with websockets.serve(echo, agent_settings["godot_websocket_ip"],
                                agent_settings['godot_websocket_port'], max_size=None,
                                max_queue=None, write_limit=None):
        await asyncio.Future()  # run forever


def websocket_operation():
    """
    WebSocket initialized to call the echo function using asyncio.
    """
    asyncio.run(main())


def action(obtained_data):
    WS_STRING = {}
    recieve_motion_data = actuators.get_motion_control_data(obtained_data)
    recieved_misc_data = actuators.get_generic_opu_data_from_feagi(obtained_data, 'misc')
    recieve_motor_data = actuators.get_motor_data(obtained_data)

    if recieve_motion_data:
        if recieve_motion_data['motion_control']:
            WS_STRING = recieve_motion_data
    if recieve_motor_data:
        WS_STRING['motor'] = {}
        for data_point in recieve_motor_data:
            WS_STRING['motor'][str(data_point)] = recieve_motor_data[data_point]
    if recieved_misc_data:
        WS_STRING['misc'] = {}
        for data_point in recieved_misc_data:
            WS_STRING['misc'][str(data_point)] = recieved_misc_data[data_point]

    if obtained_data['pointer_location']:
        WS_STRING['pointer_location'] = obtained_data['pointer_location']
    ws.append(WS_STRING)


def data_OPU(action):
    old_message = {}
    while True:
        message_from_feagi = pns.message_from_feagi
        if old_message != message_from_feagi:
            if message_from_feagi:
                if pns.full_template_information_corticals:
                    obtained_signals = pns.obtain_opu_data(message_from_feagi)
                    obtained_signals['pointer_location'] = pns.pointer_location(message_from_feagi)
                    action(obtained_signals)
        sleep(0.001)


def feagi_main(feagi_auth_url, feagi_settings, agent_settings, capabilities, message_to_feagi):
    global runtime_data
    feagi_flag = False
    print("retrying...")
    print("Waiting on FEAGI...")
    while not feagi_flag:
        feagi_flag = feagi.is_FEAGI_reachable(
            os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
            int(os.environ.get('FEAGI_OPU_PORT', "3000")))
        sleep(2)
    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    msg_counter = runtime_data["feagi_state"]['burst_counter']
    rgb = dict()
    rgb['camera'] = dict()
    previous_frame_data = {}
    default_capabilities = {}  # It will be generated in process_visual_stimuli. See the
    # overwrite manual
    default_capabilities = pns.create_runtime_default_list(default_capabilities, capabilities)
    threading.Thread(target=retina.vision_progress, args=(default_capabilities, feagi_settings, camera_data,),
                     daemon=True).start()
    actuators.start_generic_opu(capabilities)
    current_list_of_vision = pns.resize_list

    threading.Thread(target=data_OPU, args=(action,), daemon=True).start()
    actuators.start_motors(capabilities)  # initialize motors for you.

    while connected_agents['0']:
        retina.grab_visual_cortex_dimension(default_capabilities)
        # OPU section ENDS
        if camera_data['vision'] is not None:
            raw_frame = camera_data['vision']
            previous_frame_data, rgb, default_capabilities = retina.process_visual_stimuli(
                raw_frame,
                default_capabilities,
                previous_frame_data,
                rgb, capabilities)
            message_to_feagi = pns.generate_feagi_data(rgb, message_to_feagi)
        # Add accelerator section
        if 'accelerator' in acc:
            if pns.full_template_information_corticals:
                if acc['accelerator']:
                    message_to_feagi = sensors.create_data_for_feagi(sensor='accelerometer', capabilities=capabilities,
                                                                     message_to_feagi=message_to_feagi,
                                                                     current_data=acc['accelerator'], symmetric=True)

        if 'gyro' in gyro:
            if pns.full_template_information_corticals:
                if gyro['gyro']:
                    message_to_feagi = sensors.create_data_for_feagi(sensor='gyro', capabilities=capabilities,
                                                                     message_to_feagi=message_to_feagi,
                                                                     current_data=gyro['gyro'], symmetric=True,
                                                                     measure_enable=True)

        if 'proximity' in prox:
            if prox['proximity']:
                message_to_feagi = sensors.create_data_for_feagi('proximity', capabilities, message_to_feagi,
                                                                 prox['proximity'], symmetric=True)
        if current_list_of_vision != pns.resize_list:
            temp_data = retina.grab_visual_cortex_dimension(default_capabilities)
            for x in range(5):
                ws.append({'cortical_dimensions_per_device': temp_data})
            current_list_of_vision = pns.resize_list

        message_to_feagi = sensors.add_agent_status(connected_agents['0'], message_to_feagi, agent_settings)
        pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
        sleep(feagi_settings['feagi_burst_speed'])
        message_to_feagi.clear()
        if not connected_agents['0']:
            gyro.clear()
            prox.clear()


if __name__ == '__main__':
    # NEW JSON UPDATE
    configuration = feagi.build_up_from_configuration()
    feagi_settings = configuration["feagi_settings"]
    agent_settings = configuration['agent_settings']
    feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
    feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
    agent_settings['godot_websocket_port'] = os.environ.get('WS_GODOT_GENERIC_PORT', "9055")
    message_to_feagi = {"data": {}}
    # # END JSON UPDATE

    ws_operation = deque()
    acc = {}
    gyro = {}
    prox = {}
    acc['accelerator'] = {}
    prox['proximity'] = {}

    # gyro['gyro'] = []
    threading.Thread(target=websocket_operation, daemon=True).start()
    threading.Thread(target=bridge_operation, daemon=True).start()
    threading.Thread(target=godot_to_feagi, daemon=True).start()
    print("Waiting on a device to connect....")
    while not connected_agents['capabilities']:
        sleep(2)
    while True:
        while not connected_agents['capabilities']:
            sleep(0.1)  # Repeated but inside loop
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
