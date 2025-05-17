"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

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
import zlib
import asyncio
import websockets
from time import sleep
from collections import deque

queue_of_recieve_godot_data = deque()
send_to_BV_queue = deque()
current_websocket_address = deque()
current_cortical_area = {}
runtime_data = {}


async def echo(websocket):
    """
    Main thread for websocket only.
    """
    try:
        if not current_websocket_address:
            current_websocket_address.append(websocket)
        else:
            current_websocket_address[0] = websocket
        while True:
            new_data = await websocket.recv()
            decompressed_data = zlib.decompress(new_data)
            queue_of_recieve_godot_data.append(decompressed_data)
    except:
        pass



async def bridge_to_BV(runtime_data):
    while True:
        if send_to_BV_queue:
            try:
                if current_websocket_address:
                    await current_websocket_address[0].send(zlib.compress(send_to_BV_queue[0]))
                    ## TODO is this stuff below doing anything? please review
                    if "update" in send_to_BV_queue[0]:
                        send_to_BV_queue.popleft()
                    if "ping" in send_to_BV_queue:
                        send_to_BV_queue.popleft()
                    else:
                        send_to_BV_queue.pop()
                sleep(runtime_data['stimulation_period']/2)
            except Exception as error:
                sleep(0.001)
        else:
            sleep(0.001)


async def websocket_main(agent_settings):
    """
    This function sets up a WebSocket server using the 'websockets' library to communicate with a
    Godot game engine.

    The function establishes a WebSocket connection with a Godot game engine running on the
    specified IP address and port provided in 'agent_settings'. It uses the 'echo' coroutine to
    handle incoming WebSocket messages, which will echo back the received messages to the sender.

    Parameters: None

    Returns:
        None

    Raises:
        None

    Note: - The 'agent_settings' dictionary should contain the following keys: -
    'godot_websocket_ip': The IP address where websocket will broadcast for. By default,
    it should be "0.0.0.0".
    'godot_websocket_port': The port is by default to 9050. You can update in configuration.

        - The WebSocket server is configured with the following options: - 'max_size': The
        maximum size (in bytes) of incoming WebSocket messages. Set to 'None' for no limit. -
        'max_queue': The maximum number of incoming WebSocket messages that can be queued for
        processing. Set to 'None' for no limit. - 'write_limit': The maximum rate (in bytes per
        second) at which outgoing WebSocket messages can be sent. Set to 'None' for no limit. -
        'compression': The compression method to use for outgoing WebSocket messages. Set to
        'None' for no compression.

        - The function uses 'asyncio.Future()' to keep the WebSocket server running indefinitely.
        This is required because the 'websockets.serve()' coroutine itself does not naturally
        keep the server running; it only sets up the server to accept incoming connections and
        requires another coroutine or task to run the event loop.

    """
    async with websockets.serve(echo, agent_settings["godot_websocket_ip"],
                                agent_settings['godot_websocket_port'], max_size=None,
                                max_queue=None, write_limit=None):
        await asyncio.Future()


def websocket_operation(agent):
    """
    Run asyncio using websocket operations.

    This function runs the 'websocket_main()' coroutine using asyncio's 'run' function,
    which provides a simple way to execute the coroutine in the event loop.
    """
    asyncio.run(websocket_main(agent))


def bridge_operation(runtime_data):
    asyncio.run(bridge_to_BV(runtime_data))


def feagi_to_brain_visualizer(runtime_data):
    """
    Keep send_to_BV queue stay under 2 for bridge_to_BV() function. So that way, it can send latest.
    """
    while True:
        if len(send_to_BV_queue) > 0:
            if len(send_to_BV_queue) > 2:
                stored_value = send_to_BV_queue.pop()
                send_to_BV_queue.clear()
                send_to_BV_queue.append(stored_value)
            sleep(runtime_data["stimulation_period"])
        else:
            sleep(1)
