#!/usr/bin/env python
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
End-to-end tests for FEAGI's communication protocols.

This test suite verifies communication between client and server
using all FEAGI protocols: Handshake, FCP, FSMP, and FVP.
"""
import asyncio
import time

import pytest
import zmq.asyncio
from protocol.common.constants_pb2 import Timestamp
from protocol.fcp.v1.fcp_pb2 import Message as FCPMessage
from protocol.fcp.v1.fcp_pb2 import MessageType as FCPMessageType
from protocol.fcp.v1.fcp_pb2 import RegisterConfirmMessage
from protocol.fsmp.v1.fsmp_pb2 import Message as FSMPMessage
from protocol.fsmp.v1.fsmp_pb2 import MessageType as FSMPMessageType
from protocol.fvp.v1.fvp_pb2 import Message as FVPMessage
from protocol.fvp.v1.fvp_pb2 import MessageType as FVPMessageType
from protocol.handshake.v1.handshake_pb2 import ProtocolVersion

# Test Constants
HOST = "127.0.0.1"
CONTROL_PORT = 15559
SENSORIMOTOR_PORT = 15558
VIZ_PORT_BASE = 15560
SOCKET_TIMEOUT = 0.5  # Timeout for socket operations


class TestServer:
    """Mock FEAGI server for testing protocol communication."""

    def __init__(self):
        self.context = zmq.asyncio.Context()
        self.control_socket = None
        self.sensorimotor_socket = None
        self.viz_structure_socket = None
        self.viz_activity_socket = None
        self.running = False
        self.tasks = []

    async def start(self):
        """Start the test server."""
        # Control socket (ROUTER)
        self.control_socket = self.context.socket(zmq.ROUTER)
        self.control_socket.bind(f"tcp://{HOST}:{CONTROL_PORT}")

        # Sensorimotor socket (PULL+PUB)
        self.sensorimotor_socket = self.context.socket(zmq.PULL)
        self.sensorimotor_socket.bind(f"tcp://{HOST}:{SENSORIMOTOR_PORT}")
        self.sensorimotor_socket.setsockopt(zmq.RCVTIMEO, int(SOCKET_TIMEOUT * 1000))

        self.motor_pub_socket = self.context.socket(zmq.PUB)
        self.motor_pub_socket.bind(f"tcp://{HOST}:{SENSORIMOTOR_PORT}")

        # Visualization sockets
        self.viz_structure_socket = self.context.socket(zmq.PUB)
        self.viz_structure_socket.bind(f"tcp://{HOST}:{VIZ_PORT_BASE}")

        self.viz_activity_socket = self.context.socket(zmq.PUB)
        self.viz_activity_socket.bind(f"tcp://{HOST}:{VIZ_PORT_BASE + 1}")

        # Start listeners
        self.running = True
        self.tasks.append(asyncio.create_task(self._control_listener()))
        self.tasks.append(asyncio.create_task(self._sensorimotor_listener()))

    async def _control_listener(self):
        """Listen for control messages."""
        while self.running:
            try:
                # Add timeout to prevent hanging
                msg = await asyncio.wait_for(
                    self.control_socket.recv_multipart(), timeout=SOCKET_TIMEOUT
                )
                client_id, _, data = msg

                # Parse the message
                message = FCPMessage()
                message.ParseFromString(data)

                # Create timestamp
                current_time = Timestamp()
                current_time.time_ms = int(time.time() * 1000)

                # Create response based on message type
                if message.type == FCPMessageType.REGISTER_CONFIRM:
                    # Create confirmation response
                    response = FCPMessage()
                    response.type = FCPMessageType.REGISTER_CONFIRM
                    response.register_confirm.status = "active"
                    response.register_confirm.message = "Registration confirmed"
                    response.register_confirm.timestamp.CopyFrom(current_time)

                    # Send response with timeout
                    await asyncio.wait_for(
                        self.control_socket.send_multipart(
                            [client_id, b"", response.SerializeToString()]
                        ),
                        timeout=SOCKET_TIMEOUT,
                    )
            except asyncio.TimeoutError:
                # Timeout - continue loop
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Control listener error: {e}")

    async def _sensorimotor_listener(self):
        """Listen for sensory data."""
        while self.running:
            try:
                # Add timeout to prevent hanging
                data = await asyncio.wait_for(
                    self.sensorimotor_socket.recv(), timeout=SOCKET_TIMEOUT
                )

                # Parse the message
                message = FSMPMessage()
                message.ParseFromString(data)

                if message.type == FSMPMessageType.SENSORY:
                    # Create a mock motor response
                    response = FSMPMessage()
                    response.type = FSMPMessageType.MOTOR
                    response.motor_data.channel_id = 101  # Movement channel
                    response.motor_data.data = b"test_motor_data"

                    # Send motor command with timeout
                    await asyncio.wait_for(
                        self.motor_pub_socket.send_multipart(
                            [b"motor", response.SerializeToString()]
                        ),
                        timeout=SOCKET_TIMEOUT,
                    )
            except asyncio.TimeoutError:
                # Timeout - continue loop
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Sensorimotor listener error: {e}")

    async def send_visualization_data(self):
        """Send test visualization data."""
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)

        # Create structure message
        structure_msg = FVPMessage()
        structure_msg.type = FVPMessageType.STRUCTURE
        structure_msg.structure_data.timestamp.CopyFrom(current_time)

        # Add a test cortical area
        structure_msg.structure_data.cortical_areas["test_area"].id = "test_area"
        structure_msg.structure_data.cortical_areas["test_area"].name = "Test Area"

        # Send structure data with timeout
        await asyncio.wait_for(
            self.viz_structure_socket.send_multipart(
                [b"structure", structure_msg.SerializeToString()]
            ),
            timeout=SOCKET_TIMEOUT,
        )

        # Create activity message
        activity_msg = FVPMessage()
        activity_msg.type = FVPMessageType.ACTIVITY
        activity_msg.activity_data.frame_id = 1
        activity_msg.activity_data.timestamp.CopyFrom(current_time)

        # Add activity data
        test_activity = activity_msg.activity_data.activity["test_area"]
        test_activity.cortical_area_id = "test_area"
        test_activity.data = b"test_activity_data"
        test_activity.encoding_format = "binary"

        # Send activity data with timeout
        await asyncio.wait_for(
            self.viz_activity_socket.send_multipart(
                [b"activity", activity_msg.SerializeToString()]
            ),
            timeout=SOCKET_TIMEOUT,
        )

    async def stop(self):
        """Stop the test server."""
        self.running = False
        for task in self.tasks:
            task.cancel()

        # Wait for tasks to complete with timeout
        try:
            await asyncio.wait_for(
                asyncio.gather(*self.tasks, return_exceptions=True),
                timeout=SOCKET_TIMEOUT,
            )
        except asyncio.TimeoutError:
            print("Timeout while stopping server tasks")

        self.tasks = []

        # Close sockets
        for socket in [
            self.control_socket,
            self.sensorimotor_socket,
            self.motor_pub_socket,
            self.viz_structure_socket,
            self.viz_activity_socket,
        ]:
            if socket:
                socket.close(linger=0)


class TestClient:
    """Mock FEAGI client for testing protocol communication."""

    def __init__(self):
        self.context = zmq.asyncio.Context()
        self.control_socket = None
        self.sensory_socket = None
        self.motor_socket = None
        self.viz_structure_socket = None
        self.viz_activity_socket = None

        self.motor_messages = []
        self.structure_messages = []
        self.activity_messages = []

        self.tasks = []
        self.running = False

    async def connect(self):
        """Connect to test server."""
        # Create control socket (DEALER)
        self.control_socket = self.context.socket(zmq.DEALER)
        self.control_socket.connect(f"tcp://{HOST}:{CONTROL_PORT}")
        self.control_socket.setsockopt(zmq.RCVTIMEO, int(SOCKET_TIMEOUT * 1000))
        self.control_socket.setsockopt(zmq.SNDTIMEO, int(SOCKET_TIMEOUT * 1000))

        # Create sensorimotor sockets
        self.sensory_socket = self.context.socket(zmq.PUSH)
        self.sensory_socket.connect(f"tcp://{HOST}:{SENSORIMOTOR_PORT}")
        self.sensory_socket.setsockopt(zmq.SNDTIMEO, int(SOCKET_TIMEOUT * 1000))

        self.motor_socket = self.context.socket(zmq.SUB)
        self.motor_socket.connect(f"tcp://{HOST}:{SENSORIMOTOR_PORT}")
        self.motor_socket.setsockopt(zmq.SUBSCRIBE, b"motor")
        self.motor_socket.setsockopt(zmq.RCVTIMEO, int(SOCKET_TIMEOUT * 1000))

        # Create visualization sockets
        self.viz_structure_socket = self.context.socket(zmq.SUB)
        self.viz_structure_socket.connect(f"tcp://{HOST}:{VIZ_PORT_BASE}")
        self.viz_structure_socket.setsockopt(zmq.SUBSCRIBE, b"structure")
        self.viz_structure_socket.setsockopt(zmq.RCVTIMEO, int(SOCKET_TIMEOUT * 1000))

        self.viz_activity_socket = self.context.socket(zmq.SUB)
        self.viz_activity_socket.connect(f"tcp://{HOST}:{VIZ_PORT_BASE + 1}")
        self.viz_activity_socket.setsockopt(zmq.SUBSCRIBE, b"activity")
        self.viz_activity_socket.setsockopt(zmq.RCVTIMEO, int(SOCKET_TIMEOUT * 1000))

        # Start listeners
        self.running = True
        self.tasks.append(asyncio.create_task(self._motor_listener()))
        self.tasks.append(asyncio.create_task(self._structure_listener()))
        self.tasks.append(asyncio.create_task(self._activity_listener()))

    async def _motor_listener(self):
        """Listen for motor commands."""
        while self.running:
            try:
                # Add timeout to prevent hanging
                topic, data = await asyncio.wait_for(
                    self.motor_socket.recv_multipart(), timeout=SOCKET_TIMEOUT
                )
                message = FSMPMessage()
                message.ParseFromString(data)
                self.motor_messages.append(message)
            except asyncio.TimeoutError:
                # Just continue on timeout
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Motor listener error: {e}")

    async def _structure_listener(self):
        """Listen for structure updates."""
        while self.running:
            try:
                # Add timeout to prevent hanging
                topic, data = await asyncio.wait_for(
                    self.viz_structure_socket.recv_multipart(), timeout=SOCKET_TIMEOUT
                )
                message = FVPMessage()
                message.ParseFromString(data)
                self.structure_messages.append(message)
            except asyncio.TimeoutError:
                # Just continue on timeout
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Structure listener error: {e}")

    async def _activity_listener(self):
        """Listen for activity updates."""
        while self.running:
            try:
                # Add timeout to prevent hanging
                topic, data = await asyncio.wait_for(
                    self.viz_activity_socket.recv_multipart(), timeout=SOCKET_TIMEOUT
                )
                message = FVPMessage()
                message.ParseFromString(data)
                self.activity_messages.append(message)
            except asyncio.TimeoutError:
                # Just continue on timeout
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Activity listener error: {e}")

    async def send_registration(self, agent_id="test_agent", agent_type="test_type"):
        """Send registration request."""
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)

        # Create protocol versions
        versions = ProtocolVersion()
        versions.fcp_version = 1
        versions.fsmp_version = 1
        versions.fvp_version = 1

        # Create FCPMessage
        message = FCPMessage()
        message.type = FCPMessageType.REGISTER_CONFIRM

        # Create registration confirmation
        registration = RegisterConfirmMessage()
        registration.status = "active"
        registration.message = "Registration confirmed"
        registration.timestamp.CopyFrom(current_time)
        message.register_confirm.CopyFrom(registration)

        # Send message with timeout
        await asyncio.wait_for(
            self.control_socket.send_multipart([b"", message.SerializeToString()]),
            timeout=SOCKET_TIMEOUT,
        )

        # Wait for response with timeout
        response_frames = await asyncio.wait_for(
            self.control_socket.recv_multipart(), timeout=SOCKET_TIMEOUT
        )

        # Parse response
        response_message = FCPMessage()
        response_message.ParseFromString(response_frames[1])

        return response_message

    async def send_sensory_data(self, channel_id=1, data=b"test_sensory_data"):
        """Send sensory data."""
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)

        # Create FSMP sensory message
        message = FSMPMessage()
        message.type = FSMPMessageType.SENSORY
        message.sensory_data.channel_id = channel_id
        message.sensory_data.data = data
        message.sensory_data.timestamp.CopyFrom(current_time)

        # Send message with timeout
        await asyncio.wait_for(
            self.sensory_socket.send(message.SerializeToString()),
            timeout=SOCKET_TIMEOUT,
        )

    async def disconnect(self):
        """Disconnect from test server."""
        self.running = False
        for task in self.tasks:
            task.cancel()

        # Wait for tasks to complete with timeout
        try:
            await asyncio.wait_for(
                asyncio.gather(*self.tasks, return_exceptions=True),
                timeout=SOCKET_TIMEOUT,
            )
        except asyncio.TimeoutError:
            print("Timeout while stopping client tasks")

        self.tasks = []

        # Close sockets
        for socket in [
            self.control_socket,
            self.sensory_socket,
            self.motor_socket,
            self.viz_structure_socket,
            self.viz_activity_socket,
        ]:
            if socket:
                socket.close(linger=0)


# Skip this test because protocol tests need to be updated after protocol refactoring
# and Cap'n Proto has been removed from the project
@pytest.mark.skip(
    reason="Protocol tests need to be updated after protocol refactoring and CapnP removal"
)
@pytest.mark.asyncio
@pytest.mark.timeout(2)  # Shorter timeout to avoid hanging
async def test_protocol_communication():
    """Test full communication cycle using all protocols."""
    # Start server
    server = TestServer()
    await server.start()

    try:
        # Connect client
        client = TestClient()
        await client.connect()

        # Test FCP: Registration
        response = await client.send_registration()
        assert response.type == FCPMessageType.REGISTER_CONFIRM
        assert response.register_confirm.status == "active"

        # Test FSMP: Sensory data and motor command
        await client.send_sensory_data()

        # Allow time for processing - reduced time to prevent hanging
        await asyncio.sleep(0.05)

        # Verify motor command received
        assert len(client.motor_messages) > 0
        assert client.motor_messages[0].type == FSMPMessageType.MOTOR
        assert client.motor_messages[0].motor_data.channel_id == 101

        # Test FVP: Visualization data
        await server.send_visualization_data()

        # Allow time for processing - reduced time to prevent hanging
        await asyncio.sleep(0.05)

        # Verify structure data received
        assert len(client.structure_messages) > 0
        assert client.structure_messages[0].type == FVPMessageType.STRUCTURE
        assert "test_area" in client.structure_messages[0].structure_data.cortical_areas

        # Verify activity data received
        assert len(client.activity_messages) > 0
        assert client.activity_messages[0].type == FVPMessageType.ACTIVITY
        assert "test_area" in client.activity_messages[0].activity_data.activity

    finally:
        # Disconnect client
        await client.disconnect()

        # Stop server
        await server.stop()


if __name__ == "__main__":
    asyncio.run(test_protocol_communication())
