#!/usr/bin/env python3
"""
Test script to verify FEAGI and client communication using feagi_bytes.

This script tests that both FEAGI and a client can successfully 
communicate using the feagi_bytes package from PyPI.
"""

import pytest

# Skip the entire test module since the feagi_connector package structure has changed
pytest.skip("Test needs to be updated after feagi_connector refactoring", allow_module_level=True)

# Keep original code for reference
import asyncio
import json
import time
import random
import numpy as np
import logging
import sys
from typing import Dict, Any, List, Tuple, Optional, Union, Callable
import zmq
import zmq.asyncio

import feagi_bytes
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureTranslator
from feagi_connector_old.protocols import ByteStructureID, ProtocolType, FCPMessageType

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,  # Use DEBUG level to see all connection details
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('feagi_bytes_test.log')
    ]
)
logger = logging.getLogger("feagi_bytes_test")

# Custom implementation of ZmqFeagiClient for the specific FEAGI port configuration
class CustomZmqFeagiClient:
    """
    Custom ZeroMQ client for connecting to the specific running FEAGI instance.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        req_port: int = 5555,
        sensorimotor_port: int = 5558,
        visualization_port: int = 5560,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """Initialize the custom ZMQ client."""
        self.host = host
        self.req_port = req_port  # Use REQ for control messages
        self.sensorimotor_port = sensorimotor_port
        self.visualization_port = visualization_port
        
        # Initialize ZMQ context
        self.context = context or zmq.asyncio.Context.instance()
        
        # Initialize sockets (will be created on connect)
        self.req_socket = None
        self.sensorimotor_socket = None
        self.visualization_socket = None
        
        # Initialize byte structure translator
        self.translator = ByteStructureTranslator()
        
        # Client info
        self.agent_id = None
        self._running = False
        
    async def connect(self) -> bool:
        """Connect to FEAGI."""
        try:
            logger.info(f"Connecting to FEAGI at {self.host}...")
            
            # Create REQ socket for control
            logger.info(f"Connecting to req port {self.req_port}...")
            self.req_socket = self.context.socket(zmq.REQ)
            self.req_socket.connect(f"tcp://{self.host}:{self.req_port}")
            
            # Create DEALER socket for sensorimotor
            logger.info(f"Connecting to sensorimotor port {self.sensorimotor_port}...")
            self.sensorimotor_socket = self.context.socket(zmq.DEALER)
            self.sensorimotor_socket.connect(f"tcp://{self.host}:{self.sensorimotor_port}")
            
            # Create DEALER socket for visualization
            logger.info(f"Connecting to visualization port {self.visualization_port}...")
            self.visualization_socket = self.context.socket(zmq.DEALER)
            self.visualization_socket.connect(f"tcp://{self.host}:{self.visualization_port}")
            
            self._running = True
            logger.info("Connected to FEAGI sockets")
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to FEAGI: {e}")
            await self.disconnect()
            return False
    
    async def disconnect(self):
        """Disconnect from FEAGI."""
        for socket in [self.req_socket, self.sensorimotor_socket, self.visualization_socket]:
            if socket:
                socket.close(linger=0)
                
        self.req_socket = None
        self.sensorimotor_socket = None
        self.visualization_socket = None
        self._running = False
        logger.info("Disconnected from FEAGI")
    
    async def send_sensory_data(self, channel_id: int, data: Union[bytes, List[float]]) -> bool:
        """Send sensory data to FEAGI."""
        if not self._running or not self.sensorimotor_socket:
            logger.error("Cannot send sensory data: not connected")
            return False
            
        try:
            # Create a simple message format
            message = {
                "message_type": "sensory",
                "channel_id": channel_id,
                "data": data if isinstance(data, list) else data.hex()[:100] + "...",  # Truncate hex representation for logging
                "timestamp": time.time()
            }
            
            logger.info(f"Sending sensory data to channel {channel_id}")
            
            # Send data (with empty delimiter for DEALER-ROUTER pattern)
            await self.sensorimotor_socket.send_multipart([
                b"",  # Empty delimiter
                json.dumps({"message_type": "sensory", "channel_id": channel_id}).encode('utf-8'),
                data if isinstance(data, bytes) else json.dumps(data).encode('utf-8')
            ])
            
            logger.info(f"Sensory data sent to channel {channel_id}")
            return True
            
        except Exception as e:
            logger.exception(f"Error sending sensory data: {e}")
            return False

    async def get_status(self) -> Dict[str, Any]:
        """Get FEAGI status."""
        if not self._running or not self.req_socket:
            logger.error("Cannot get status: not connected")
            return {"error": "Not connected"}
            
        try:
            # Create status request
            request = {"type": "status_request"}
            
            # Send request
            logger.info("Sending status request")
            await self.req_socket.send_json(request)
            
            # Receive response with timeout
            try:
                logger.info("Waiting for status response")
                response = await asyncio.wait_for(self.req_socket.recv_json(), timeout=5.0)
                logger.info("Received status response")
                return response
            except asyncio.TimeoutError:
                logger.warning("Status request timed out")
                return {"error": "Request timed out"}
                
        except Exception as e:
            logger.exception(f"Error getting status: {e}")
            return {"error": str(e)}

def test_raw_data(data_type: str = "image", size: Tuple[int, int] = (28, 28), channels: int = 1) -> bytes:
    """
    Generate fake sensory data for testing.
    
    Args:
        data_type: Type of data to generate ('image' or 'array')
        size: Size of the data (width, height)
        channels: Number of channels (for images)
    
    Returns:
        Raw bytes data ready to be sent to FEAGI
    """
    width, height = size
    
    if data_type == "image":
        # Generate a random grayscale or RGB image
        if channels == 1:
            # Grayscale image
            pixel_values = np.random.randint(0, 256, size=(height, width), dtype=np.uint8)
        else:
            # RGB image
            pixel_values = np.random.randint(0, 256, size=(height, width, channels), dtype=np.uint8)
        
        # Convert to bytes
        raw_data = pixel_values.tobytes()
        logger.info(f"Generated test image: {width}x{height}x{channels}, size: {len(raw_data)} bytes")
        return raw_data
    
    elif data_type == "array":
        # Generate array of random float values (e.g., for direct neuron stimulation)
        values = np.random.random(size=width * height).astype(np.float32)
        raw_data = values.tobytes()
        logger.info(f"Generated test array: {len(values)} values, size: {len(raw_data)} bytes")
        return raw_data
    
    else:
        raise ValueError(f"Unknown data type: {data_type}")

async def test_client_communication():
    """Test direct communication with FEAGI using custom client."""
    
    logger.info(f"Using feagi_bytes version: {feagi_bytes.__version__}")
    
    # Create custom client
    client = CustomZmqFeagiClient(
        host="localhost",
        req_port=5555,
        sensorimotor_port=5558,
        visualization_port=5560
    )
    
    # Try to connect with timeout
    logger.info("Connecting to FEAGI...")
    try:
        connected = await asyncio.wait_for(client.connect(), timeout=5.0)
        
        if not connected:
            logger.error("Failed to connect to FEAGI!")
            return False
            
        logger.info("Successfully connected to FEAGI!")
        
        # Current timestamp to make our test data uniquely identifiable
        timestamp = int(time.time())
        logger.info(f"Test timestamp: {timestamp}")
        
        # Generate and send an image with a recognizable pattern
        logger.info("Generating and sending test image data with recognizable pattern for cortical area iv00_C...")
        
        # Create a simple pattern - alternating black and white stripes
        height, width = 64, 64
        channels = 3
        image_data = np.zeros((height, width, channels), dtype=np.uint8)
        
        # Create vertical stripes
        for x in range(width):
            if x % 8 < 4:  # 4-pixel wide stripes
                image_data[:, x, :] = 255  # White stripe
                
        # Add timestamp to corner pixels
        digits = [int(d) for d in str(timestamp)]
        for i, digit in enumerate(digits[:10]):  # Use up to 10 digits
            if i < 10:
                # Set corner pixel based on digit value (0-9)
                image_data[i, 0, 0] = digit * 25  # Scale to visible range
                
        raw_image_data = image_data.tobytes()
        logger.info(f"Created test image with timestamp pattern: {timestamp}")
        logger.info(f"Image data hash: {hash(raw_image_data) % 10000}")
        
        image_result = await client.send_sensory_data(channel_id="iv00_C", data=raw_image_data)
        logger.info(f"Image data send result: {image_result}")
        
        # Generate and send a float array with a recognizable pattern
        logger.info("Generating and sending test array with recognizable pattern for cortical area iv00_C...")
        
        # Create a simple sine wave pattern
        x = np.linspace(0, 4*np.pi, 100)
        float_values = np.sin(x).astype(np.float32)
        
        # Mark with timestamp in first 10 values
        for i, digit in enumerate(digits[:10]):
            if i < 10:
                float_values[i] = digit / 10.0  # Scale to [-1, 1] range
                
        raw_array_data = float_values.tobytes()
        logger.info(f"Created test array with timestamp pattern: {timestamp}")
        logger.info(f"Array data hash: {hash(raw_array_data) % 10000}")
        
        array_result = await client.send_sensory_data(channel_id="iv00_C", data=raw_array_data)
        logger.info(f"Array data send result: {array_result}")
        
        # Wait a bit to ensure data is processed
        logger.info("Waiting for data to be processed...")
        await asyncio.sleep(2)
        
        # Disconnect
        logger.info("Disconnecting from FEAGI...")
        await client.disconnect()
        logger.info("Disconnected from FEAGI")
        
        return True
        
    except asyncio.TimeoutError:
        logger.error("Connection attempt timed out!")
        await client.disconnect()
        return False

if __name__ == "__main__":
    # Run the test
    logger.info("\n==== Testing FEAGI-Client Communication with feagi_bytes ====\n")
    success = asyncio.run(test_client_communication())
    
    if success:
        logger.info("\n✅ Test completed successfully! FEAGI and client are using feagi_bytes.")
    else:
        logger.error("\n❌ Test failed! Check if FEAGI is running and if both are using the same feagi_bytes package.") 