#!/usr/bin/env python
"""
End-to-end tests for FEAGI's communication protocols.

This test suite verifies communication between client and server
using all FEAGI protocols: Handshake, FCP, FSMP, and FVP.
"""
import asyncio
import time
import pytest
import zmq.asyncio

# Import our protocol enums instead of the protobuf ones
from feagi.api.protocols.constants import ProtocolID, FCPCommandType, FSMPChannelType, FVPFrameType
from feagi.api.protocols.fcp import FCPv1
from feagi.api.protocols.fsmp import FSMPv1
from feagi.api.protocols.fvp import FVPv1

# Simulate protocol messages with dictionaries instead of protobuf objects
class Timestamp:
    @classmethod
    def create(cls):
        return {"time_ms": int(time.time() * 1000)}

# Test Constants
HOST = "127.0.0.1"
CONTROL_PORT = 15559
SENSORIMOTOR_PORT = 15558
VIZ_PORT_BASE = 15560


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
                msg = await self.control_socket.recv_multipart()
                client_id, _, data = msg
                
                # Parse the message
                message = FCPv1.Message()
                message.ParseFromString(data)
                
                # Create timestamp
                current_time = Timestamp.create()
                
                # Create response based on message type
                if message.type == FCPCommandType.REGISTER_CONFIRM:
                    # Create confirmation response
                    response = FCPv1.Message()
                    response.type = FCPCommandType.REGISTER_CONFIRM
                    response.register_confirm.status = "active"
                    response.register_confirm.message = "Registration confirmed"
                    response.register_confirm.timestamp = current_time
                    
                    # Send response
                    await self.control_socket.send_multipart([
                        client_id, 
                        b"", 
                        response.SerializeToString()
                    ])
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Control listener error: {e}")
    
    async def _sensorimotor_listener(self):
        """Listen for sensory data."""
        while self.running:
            try:
                data = await self.sensorimotor_socket.recv()
                
                # Parse the message
                message = FSMPv1.Message()
                message.ParseFromString(data)
                
                if message.type == FSMPChannelType.SENSORY:
                    # Create a mock motor response
                    response = FSMPv1.Message()
                    response.type = FSMPChannelType.MOTOR
                    response.motor_data.channel_id = 101  # Movement channel
                    response.motor_data.data = b"test_motor_data"
                    
                    # Send motor command
                    await self.motor_pub_socket.send_multipart([
                        b"motor",
                        response.SerializeToString()
                    ])
            
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Sensorimotor listener error: {e}")
    
    async def send_visualization_data(self):
        """Send test visualization data."""
        # Create timestamp
        current_time = Timestamp.create()
        
        # Create structure message
        structure_msg = FVPv1.Message()
        structure_msg.type = FVPFrameType.STRUCTURE
        structure_msg.structure_data.timestamp = current_time
        
        # Add a test cortical area
        structure_msg.structure_data.cortical_areas["test_area"] = {
            "id": "test_area",
            "name": "Test Area"
        }
        
        # Send structure data
        await self.viz_structure_socket.send_multipart([
            b"structure",
            structure_msg.SerializeToString()
        ])
        
        # Create activity message
        activity_msg = FVPv1.Message()
        activity_msg.type = FVPFrameType.ACTIVITY
        activity_msg.activity_data.frame_id = 1
        activity_msg.activity_data.timestamp = current_time
        
        # Add activity data
        test_activity = activity_msg.activity_data.activity["test_area"] = {
            "cortical_area_id": "test_area",
            "data": b"test_activity_data",
            "encoding_format": "binary"
        }
        
        # Send activity data
        await self.viz_activity_socket.send_multipart([
            b"activity",
            activity_msg.SerializeToString()
        ])
    
    async def stop(self):
        """Stop the test server."""
        self.running = False
        for task in self.tasks:
            task.cancel()
        
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.tasks = []
        
        # Close sockets
        for socket in [self.control_socket, self.sensorimotor_socket, 
                      self.motor_pub_socket, self.viz_structure_socket, 
                      self.viz_activity_socket]:
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
        
        # Create sensorimotor sockets
        self.sensory_socket = self.context.socket(zmq.PUSH)
        self.sensory_socket.connect(f"tcp://{HOST}:{SENSORIMOTOR_PORT}")
        
        self.motor_socket = self.context.socket(zmq.SUB)
        self.motor_socket.connect(f"tcp://{HOST}:{SENSORIMOTOR_PORT}")
        self.motor_socket.setsockopt(zmq.SUBSCRIBE, b"motor")
        
        # Create visualization sockets
        self.viz_structure_socket = self.context.socket(zmq.SUB)
        self.viz_structure_socket.connect(f"tcp://{HOST}:{VIZ_PORT_BASE}")
        self.viz_structure_socket.setsockopt(zmq.SUBSCRIBE, b"structure")
        
        self.viz_activity_socket = self.context.socket(zmq.SUB)
        self.viz_activity_socket.connect(f"tcp://{HOST}:{VIZ_PORT_BASE + 1}")
        self.viz_activity_socket.setsockopt(zmq.SUBSCRIBE, b"activity")
        
        # Start listeners
        self.running = True
        self.tasks.append(asyncio.create_task(self._motor_listener()))
        self.tasks.append(asyncio.create_task(self._structure_listener()))
        self.tasks.append(asyncio.create_task(self._activity_listener()))
    
    async def _motor_listener(self):
        """Listen for motor commands."""
        while self.running:
            try:
                topic, data = await self.motor_socket.recv_multipart()
                message = FSMPv1.Message()
                message.ParseFromString(data)
                self.motor_messages.append(message)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Motor listener error: {e}")
    
    async def _structure_listener(self):
        """Listen for structure updates."""
        while self.running:
            try:
                topic, data = await self.viz_structure_socket.recv_multipart()
                message = FVPv1.Message()
                message.ParseFromString(data)
                self.structure_messages.append(message)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Structure listener error: {e}")
    
    async def _activity_listener(self):
        """Listen for activity updates."""
        while self.running:
            try:
                topic, data = await self.viz_activity_socket.recv_multipart()
                message = FVPv1.Message()
                message.ParseFromString(data)
                self.activity_messages.append(message)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Activity listener error: {e}")
    
    async def send_registration(self, agent_id="test_agent", agent_type="test_type"):
        """Send registration request."""
        # Create timestamp
        current_time = Timestamp.create()
        
        # Create protocol versions
        versions = {
            "fcp_version": 1,
            "fsmp_version": 1,
            "fvp_version": 1
        }
        
        # Create FCPMessage
        message = FCPv1.Message()
        message.type = FCPCommandType.REGISTER_CONFIRM
        
        # Create registration confirmation
        registration = {
            "status": "active",
            "message": "Registration confirmed",
            "timestamp": current_time
        }
        message.register_confirm.CopyFrom(registration)
        
        # Send message
        await self.control_socket.send_multipart([b"", message.SerializeToString()])
        
        # Wait for response
        response_frames = await self.control_socket.recv_multipart()
        
        # Parse response
        response_message = FCPv1.Message()
        response_message.ParseFromString(response_frames[1])
        
        return response_message
    
    async def send_sensory_data(self, channel_id=1, data=b"test_sensory_data"):
        """Send sensory data."""
        # Create timestamp
        current_time = Timestamp.create()
        
        # Create FSMP sensory message
        message = FSMPv1.Message()
        message.type = FSMPChannelType.SENSORY
        message.sensory_data.channel_id = channel_id
        message.sensory_data.data = data
        message.sensory_data.timestamp = current_time
        
        # Send message
        await self.sensory_socket.send(message.SerializeToString())
    
    async def disconnect(self):
        """Disconnect from test server."""
        self.running = False
        for task in self.tasks:
            task.cancel()
        
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.tasks = []
        
        # Close sockets
        for socket in [self.control_socket, self.sensory_socket, self.motor_socket,
                      self.viz_structure_socket, self.viz_activity_socket]:
            if socket:
                socket.close(linger=0)


@pytest.mark.skip(reason="Requires protobuf implementation of Message classes")
@pytest.mark.asyncio
async def test_protocol_communication():
    """Test end-to-end communication using all protocols."""
    # Start server
    server = TestServer()
    await server.start()
    
    try:
        # Connect client
        client = TestClient()
        await client.connect()
        
        # Test FCP: Registration
        response = await client.send_registration()
        assert response.type == FCPCommandType.REGISTER_CONFIRM
        assert response.register_confirm.status == "active"
        
        # Test FSMP: Sensory data and motor command
        await client.send_sensory_data()
        
        # Allow time for processing
        await asyncio.sleep(0.5)
        
        # Verify motor command received
        assert len(client.motor_messages) > 0
        assert client.motor_messages[0].type == FSMPChannelType.MOTOR
        assert client.motor_messages[0].motor_data.channel_id == 101
        
        # Test FVP: Visualization data
        await server.send_visualization_data()
        
        # Allow time for processing
        await asyncio.sleep(0.5)
        
        # Verify structure data received
        assert len(client.structure_messages) > 0
        assert client.structure_messages[0].type == FVPFrameType.STRUCTURE
        assert "test_area" in client.structure_messages[0].structure_data.cortical_areas
        
        # Verify activity data received
        assert len(client.activity_messages) > 0
        assert client.activity_messages[0].type == FVPFrameType.ACTIVITY
        assert "test_area" in client.activity_messages[0].activity_data.activity
        
    finally:
        # Disconnect client
        await client.disconnect()
        
        # Stop server
        await server.stop()


if __name__ == "__main__":
    asyncio.run(test_protocol_communication()) 