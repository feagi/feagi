#!/usr/bin/env python
"""
Example client using Cap'n Proto for FEAGI communication.

This example demonstrates how to use the Cap'n Proto schema definitions
to communicate with a FEAGI server.
"""

import asyncio
import logging
import time
import os
import sys

# Add parent directory to path to import capnp module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zmq
import zmq.asyncio
from capnp import KjException

# Import Cap'n Proto schemas
import capnp
from capnp.lib import capnp as capnp_lib

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_client_example")

# Constants
HOST = "localhost"
CONTROL_PORT = 5559
SENSORIMOTOR_PORT = 5558
VIZ_PORT_BASE = 5560
AGENT_ID = "example-capnp-client"
AGENT_TYPE = "example"


class CapnpFeagiClient:
    """Example client using Cap'n Proto for FEAGI communication."""
    
    def __init__(self):
        """Initialize the client."""
        self.context = zmq.asyncio.Context()
        self.control_socket = None
        
        # Load Cap'n Proto schemas
        self.schema_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "feagi_capnp")
        
        try:
            self.constants_schema = capnp.load(os.path.join(self.schema_path, "common/constants.capnp"))
            self.handshake_schema = capnp.load(os.path.join(self.schema_path, "handshake/v1/handshake.capnp"))
            self.fcp_schema = capnp.load(os.path.join(self.schema_path, "fcp/v1/fcp.capnp"))
            self.fsmp_schema = capnp.load(os.path.join(self.schema_path, "fsmp/v1/fsmp.capnp"))
            self.fvp_schema = capnp.load(os.path.join(self.schema_path, "fvp/v1/fvp.capnp"))
            logger.info("Cap'n Proto schemas loaded successfully")
        except (KjException, capnp_lib.KjException) as e:
            logger.error(f"Failed to load Cap'n Proto schemas: {e}")
            raise
    
    async def connect(self):
        """Connect to the FEAGI server."""
        logger.info(f"Connecting to FEAGI server at {HOST}:{CONTROL_PORT}")
        
        # Create control socket (DEALER for ROUTER)
        self.control_socket = self.context.socket(zmq.DEALER)
        self.control_socket.connect(f"tcp://{HOST}:{CONTROL_PORT}")
        logger.info("Connected to control socket")
        
        # Perform handshake
        await self.handshake()
    
    async def handshake(self):
        """Perform the handshake protocol with FEAGI."""
        logger.info("Performing handshake")
        
        # Create timestamp
        current_time = self.constants_schema.Timestamp.new_message(timeMs=int(time.time() * 1000))
        
        # Create hello message
        hello = self.handshake_schema.HelloMessage.new_message(
            agentId=AGENT_ID,
            agentType=AGENT_TYPE
        )
        hello.timestamp = current_time
        
        # Create handshake message
        handshake_msg = self.handshake_schema.HandshakeMessage.new_message(
            protocolId=self.constants_schema.ProtocolID.fcp,
            version=1,
            type=self.handshake_schema.HandshakeMessageType.hello
        )
        handshake_msg.hello = hello
        
        # Serialize and send
        data = handshake_msg.to_bytes()
        await self.control_socket.send_multipart([b"", data])
        logger.info("Sent hello message")
        
        # Handle welcome response
        response_frames = await self.control_socket.recv_multipart()
        if len(response_frames) != 2:
            logger.error(f"Invalid response format: {response_frames}")
            return False
            
        # Deserialize response
        try:
            response = self.handshake_schema.HandshakeMessage.from_bytes(response_frames[1])
            logger.info(f"Received {response.type} message from server")
            
            if response.type == self.handshake_schema.HandshakeMessageType.welcome:
                logger.info(f"Server ID: {response.welcome.serverId}")
                logger.info(f"Message: {response.welcome.message}")
                
                # Continue with capabilities
                await self.send_capabilities()
                
            else:
                logger.error(f"Unexpected response type: {response.type}")
                return False
                
        except (KjException, capnp_lib.KjException) as e:
            logger.error(f"Failed to parse response: {e}")
            return False
    
    async def send_capabilities(self):
        """Send capabilities to the server."""
        logger.info("Sending capabilities")
        
        # Create timestamp
        current_time = self.constants_schema.Timestamp.new_message(timeMs=int(time.time() * 1000))
        
        # Create protocol versions
        protocol_versions = self.handshake_schema.ProtocolVersion.new_message(
            fcpVersion=1,
            fsmpVersion=1,
            fvpVersion=1
        )
        
        # Create capabilities message
        capabilities = self.handshake_schema.CapabilitiesMessage.new_message(
            supportedSensoryChannels=[1, 2],  # Vision, Audio
            supportedMotorChannels=[101]      # Movement
        )
        capabilities.protocolVersions = protocol_versions
        capabilities.timestamp = current_time
        
        # Add features
        feature1 = capabilities.init("features", 1)[0]
        feature1.name = "streaming"
        feature1.enabled = True
        
        # Create handshake message
        handshake_msg = self.handshake_schema.HandshakeMessage.new_message(
            protocolId=self.constants_schema.ProtocolID.fcp,
            version=1,
            type=self.handshake_schema.HandshakeMessageType.capabilities
        )
        handshake_msg.capabilities = capabilities
        
        # Serialize and send
        data = handshake_msg.to_bytes()
        await self.control_socket.send_multipart([b"", data])
        logger.info("Sent capabilities message")
        
        # Handle configuration response (we would continue the handshake here)
        logger.info("Handshake completed successfully")
    
    async def disconnect(self):
        """Disconnect from the FEAGI server."""
        logger.info("Disconnecting from FEAGI server")
        
        if self.control_socket:
            self.control_socket.close()
            self.control_socket = None


async def main():
    """Main function to run the example client."""
    client = CapnpFeagiClient()
    try:
        await client.connect()
        
        # Keep the client running for a bit
        await asyncio.sleep(1)
        
    finally:
        await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main()) 