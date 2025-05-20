"""
FEAGI Sensory Client

Client for sending sensory data to FEAGI and receiving motor data using ZMQ DEALER/ROUTER pattern.
"""

import json
import logging
import uuid
import asyncio
from typing import Dict, Any, Optional, List, Union, Tuple

import zmq
import zmq.asyncio
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureID

# Import constants
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ

# Import default implementation - this should be the Python implementation
# for backward compatibility
from feagi_connector.utils.processing import (
    infer_byte_structure_type_python as infer_byte_structure_type,
    decode_neuron_potential_xyz_python as decode_neuron_potential_xyz
)

# Configure logging
logger = logging.getLogger("feagi_connector.sensory")


class FeagiSensoryClient:
    """
    Client for sending sensory data to FEAGI using DEALER/ROUTER pattern (port 5558).
    
    This client properly handles binary data transmission for sensory input,
    using the correct message framing for DEALER/ROUTER.
    """
    
    def __init__(
        self, 
        host: str = "127.0.0.1", 
        port: int = 5558, 
        agent_id: Optional[str] = None, 
        socket_timeout: int = 1000
    ):
        """
        Initialize the sensory client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ DEALER/ROUTER port (default 5558)
            agent_id: Agent ID for FEAGI registration (default: auto-generated)
            socket_timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.agent_id = agent_id or f"agent-{uuid.uuid4().hex[:8]}"
        self.timeout = socket_timeout
        self.context = zmq.asyncio.Context.instance()
        self.socket = None
        self.registered = False
        
    async def connect(self) -> bool:
        """
        Create and connect a socket.
        
        Returns:
            True if connection was successful
        """
        try:
            if self.socket:
                self.socket.close()
                
            self.socket = self.context.socket(zmq.DEALER)
            self.socket.setsockopt(zmq.IDENTITY, self.agent_id.encode())
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait for pending messages on close
            
            # Set a connect timeout
            self.socket.setsockopt(zmq.CONNECT_TIMEOUT, 1000)  # 1 second timeout
            
            logger.debug(f"Connecting to sensorimotor server at tcp://{self.host}:{self.port}")
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            logger.debug(f"Connected to sensorimotor server")
            return True
            
        except zmq.error.ZMQError as e:
            logger.error(f"ZMQ Error connecting to {self.host}:{self.port}: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            return False
            
    async def close(self) -> None:
        """Close the socket."""
        if self.socket:
            self.socket.close()
            self.socket = None
            self.registered = False
    
    async def register_agent(
        self, 
        agent_type: str = "external", 
        sensory_channels: Optional[List[int]] = None, 
        motor_channels: Optional[List[int]] = None
    ) -> bool:
        """
        Register this agent with FEAGI.
        
        Args:
            agent_type: Type of agent 
            sensory_channels: List of sensory channels
            motor_channels: List of motor channels
            
        Returns:
            True if registration was successful
        """
        if self.registered:
            return True
           
        # Make sure we're connected
        if not self.socket:
            if not await self.connect():
                return False
            
        # Create registration message
        hello_msg = {
            "message_type": "hello",
            "agent_id": self.agent_id,
            "agent_type": agent_type,
            "sensory_channels": sensory_channels or [],
            "motor_channels": motor_channels or []
        }
        
        try:
            # Send hello message
            logger.debug(f"Sending hello message: {hello_msg}")
            await self.socket.send_multipart([
                b"",  # Empty delimiter frame required for ROUTER
                json.dumps(hello_msg).encode('utf-8')
            ])
            
            # Wait for welcome message
            try:
                welcome_frames = await self.socket.recv_multipart()
                
                if len(welcome_frames) < 2:
                    logger.error(f"Invalid welcome message format: {welcome_frames}")
                    return False
                
                # Skip the empty delimiter frame
                welcome_data = welcome_frames[1]
                
                try:
                    welcome_msg = json.loads(welcome_data.decode('utf-8'))
                    
                    if welcome_msg.get("message_type") == "welcome":
                        logger.info(f"Successfully registered agent {self.agent_id} with FEAGI")
                        self.registered = True
                        return True
                    else:
                        logger.error(f"Unexpected response to hello: {welcome_msg}")
                        return False
                        
                except json.JSONDecodeError:
                    logger.error(f"Could not decode welcome message: {welcome_data}")
                    return False
                    
            except zmq.error.Again:
                logger.error("Timed out waiting for welcome message")
                return False
                
        except Exception as e:
            logger.error(f"Error registering agent: {e}")
            return False
    
    async def send_sensory_data(
        self, 
        cortical_area: Union[str, int], 
        data: Union[bytes, Dict[Tuple[int, int, int], float]]
    ) -> bool:
        """
        Send sensory data to FEAGI.
        
        Args:
            cortical_area: Cortical area ID or name
            data: Sensory data as bytes or neuron activation dictionary
            
        Returns:
            True if data was sent successfully
        """
        if not self.socket or not self.registered:
            logger.error("Not connected or registered")
            return False
        
        try:
            # Prepare the message header
            header = {
                "message_type": "sensory_data",
                "cortical_area": cortical_area,
                "timestamp": int(asyncio.get_event_loop().time() * 1000)
            }
            
            # Encode the data
            if isinstance(data, bytes):
                # Raw binary data
                binary_data = data
            elif isinstance(data, dict):
                # Dictionary of neuron coordinates to activation values
                encoder = ByteStructureEncoder()
                binary_data = encoder.encode_neuron_data(data)
            else:
                logger.error(f"Unsupported data type: {type(data)}")
                return False
            
            # Send the message
            logger.debug(f"Sending sensory data to {cortical_area}: {len(binary_data)} bytes")
            await self.socket.send_multipart([
                b"",  # Empty delimiter frame required for ROUTER
                json.dumps(header).encode('utf-8'),
                binary_data
            ])
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False
    
    async def process_motor_data(self, data: bytes) -> Optional[Dict]:
        """
        Process motor data received from FEAGI.
        
        Args:
            data: Raw binary data from FEAGI
            
        Returns:
            Processed motor data or None if processing failed
        """
        try:
            # Determine the data type
            data_type = infer_byte_structure_type(data)
            
            if data_type == NEURON_POTENTIAL_CATEGORICAL_XYZ:
                # Decode neuron potential data
                neuron_data = decode_neuron_potential_xyz(data)
                    
                return {
                    "type": "neuron_potential",
                    "format": "xyz",
                    "neurons": neuron_data
                }
            else:
                logger.warning(f"Unsupported data type: {data_type}")
                return None
                
        except Exception as e:
            logger.error(f"Error processing motor data: {e}")
            return None
    
    async def receive_motor_data(self, timeout: int = None) -> Optional[Tuple[int, bytes]]:
        """
        Receive motor data from FEAGI.
        
        Args:
            timeout: Receive timeout in milliseconds (None for default)
            
        Returns:
            Tuple of (channel_id, data) or None if no data available
        """
        if not self.socket or not self.registered:
            logger.error("Not connected or registered")
            return None
        
        try:
            # Set timeout if provided
            if timeout is not None:
                old_timeout = self.socket.getsockopt(zmq.RCVTIMEO)
                self.socket.setsockopt(zmq.RCVTIMEO, timeout)
            
            # Try to receive a message
            try:
                frames = await self.socket.recv_multipart()
                
                if len(frames) < 3:
                    logger.warning(f"Invalid motor data format: {frames}")
                    return None
                
                # Skip the empty delimiter frame
                header_data = frames[1]
                motor_data = frames[2]
                
                try:
                    header = json.loads(header_data.decode('utf-8'))
                    
                    if header.get("message_type") == "motor_data":
                        channel_id = header.get("channel_id")
                        logger.debug(f"Received motor data on channel {channel_id}: {len(motor_data)} bytes")
                        processed_data = await self.process_motor_data(motor_data)
                        return channel_id, processed_data
                    else:
                        logger.warning(f"Unexpected message type: {header.get('message_type')}")
                        return None
                        
                except json.JSONDecodeError:
                    logger.error(f"Could not decode motor data header: {header_data}")
                    return None
                    
            except zmq.error.Again:
                # No data available within timeout
                return None
                
        except Exception as e:
            logger.error(f"Error receiving motor data: {e}")
            return None
            
        finally:
            # Reset timeout if it was changed
            if timeout is not None:
                self.socket.setsockopt(zmq.RCVTIMEO, old_timeout)
    
    async def send_heartbeat(self) -> bool:
        """
        Send a heartbeat to keep the connection alive.
        
        Returns:
            True if heartbeat was sent successfully
        """
        if not self.socket or not self.registered:
            logger.error("Not connected or registered")
            return False
        
        try:
            heartbeat_msg = {
                "message_type": "heartbeat",
                "agent_id": self.agent_id,
                "timestamp": int(asyncio.get_event_loop().time() * 1000)
            }
            
            await self.socket.send_multipart([
                b"",  # Empty delimiter frame required for ROUTER
                json.dumps(heartbeat_msg).encode('utf-8')
            ])
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending heartbeat: {e}")
            return False 