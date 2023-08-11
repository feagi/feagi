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
import zmq.asyncio
import websockets

"""
Create two separate tasks: one for handling messages from Godot (WebSocket) and another for handling messages from FEAGI (ZMQ).
Use an asyncio.Semaphore to ensure that when a Godot message is being processed, FEAGI messages are temporarily paused.
Periodically check (or "poll") the WebSocket for Godot messages even when processing a batch of FEAGI messages to ensure Godot messages aren't left waiting for too long.
"""

# Configuration variables
ZMQ_ADDRESS = "tcp://localhost:xxxxx"  # FEAGI server's ZMQ address
WEBSOCKET_ADDRESS = "ws://localhost:9050"  # Godot's WebSocket address

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
        await semaphore.acquire()
        try:
            await zmq_socket.send_string(msg)
        finally:
            semaphore.release()


async def main():
    async with websockets.connect(WEBSOCKET_ADDRESS) as ws:
        forwarder_tasks = [
            forward_from_zmq_to_ws(ws),
            forward_from_ws_to_zmq(ws)
        ]
        await asyncio.gather(*forwarder_tasks)

# Run the main function
asyncio.run(main())
