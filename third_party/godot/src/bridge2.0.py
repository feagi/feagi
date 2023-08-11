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

import asyncio
import zmq
import gzip
import zmq.asyncio
import websockets
import threading
from configuration import agent_settings, feagi_settings
from feagi_agent import feagi_interface as feagi

"""
Create two separate tasks: one for handling messages from Godot (WebSocket) and another for handling messages from FEAGI (ZMQ).
Use an asyncio.Semaphore to ensure that when a Godot message is being processed, FEAGI messages are temporarily paused.
Periodically check (or "poll") the WebSocket for Godot messages even when processing a batch of FEAGI messages to ensure Godot messages aren't left waiting for too long.
"""

runtime_data = {
    "cortical_data": {},
    "current_burst_id": None,
    "stimulation_period": None,
    "feagi_state": None,
    "feagi_network": None,
    "cortical_list": set(),
    "host_network": {},
    "genome_number": 0,
    "old_cortical_data": {}
}

# Configuration variables
ZMQ_ADDRESS = "tcp://localhost:3000"  # FEAGI server's ZMQ address
WEBSOCKET_SERVER_ADDRESS = "localhost"
WEBSOCKET_SERVER_PORT = 9050  # Port on which the WebSocket server will listen

# Initialize ZMQ components
zmq_context = zmq.asyncio.Context()
zmq_socket = zmq_context.socket(zmq.PAIR)
zmq_socket.connect(ZMQ_ADDRESS)

semaphore = asyncio.Semaphore(1)


async def forward_from_zmq_to_ws(ws):
    while True:
        # Wait for semaphore, ensuring Godot messages aren't being processed
        await semaphore.acquire()
        try:
            msg = await zmq_socket.recv_string()
            await ws.send(msg)
        finally:
            semaphore.release()
        await asyncio.sleep(0.001)  # Give a small break to allow Godot message processing


async def forward_from_ws_to_zmq(ws):
    while True:
        msg = await ws.recv()
        decompressed_data = gzip.decompress(msg)
        print("msg: ", decompressed_data)


async def ws_handler(websocket, path):
    forwarder_tasks = [
        forward_from_zmq_to_ws(websocket),
        forward_from_ws_to_zmq(websocket)
    ]
    await asyncio.gather(*forwarder_tasks)


def websocket_operation():
    start_server = websockets.serve(ws_handler, WEBSOCKET_SERVER_ADDRESS, WEBSOCKET_SERVER_PORT)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


# if __name__ == "__main__":
#     # FEAGI section start
#     print("Connecting to FEAGI resources...")
#     feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
#     print("FEAGI AUTH URL ------- ", feagi_auth_url)
#     feagi_host, api_port, app_data_port = feagi.feagi_setting_for_registration(feagi_settings,
#                                                                                agent_settings)
#     print("app_data_port: ", app_data_port)
#     capabilities = {}
#     runtime_data["feagi_state"] = feagi.feagi_registration(feagi_auth_url=feagi_auth_url,
#                                                            feagi_settings=feagi_settings,
#                                                            agent_settings=agent_settings,
#                                                            capabilities=capabilities)
#     agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
#     print("** **", runtime_data["feagi_state"])
#     feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])
#     ipu_channel_address = "tcp://*:" + agent_data_port
#     print("ipu: ", ipu_channel_address)
#     opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
#                                                runtime_data["feagi_state"]['feagi_opu_port'])
#     feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=True)
#     # FEAGI section ends
#
#     print("FEAGI initialization completed successfully")
#
#     # Dict and Ararys initalizion #
#     godot_list = {}  # initialized the list from Godot
#
#     detect_lag = False
#     new_feagi_sub = feagi.sub_initializer(opu_address=opu_channel_address)
#     flag_zmq = False
#     connect_status_counter = 0
#     old_data = []
#     threading.Thread(target=websocket_operation, daemon=True).start()
#
#     # TEST ZMQ RECEIVER!!
#     received_data = new_feagi_sub.receive()
#     if received_data is not None:
#         if isinstance(received_data, bytes):
#             decompressed_data = lz4.frame.decompress(received_data)
#             one_frame = pickle.loads(decompressed_data)
#         else:
#             one_frame = received_data
#     else:
#         one_frame = None
#     print(received_data)

# # Run the main function
asyncio.run(websocket_operation())
