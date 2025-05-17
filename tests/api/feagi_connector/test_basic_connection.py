#!/usr/bin/env python
"""
Basic test for the FEAGI connector with the byte structure protocol.

This script tests the basic connectivity with a running FEAGI instance.
All communication is done using binary data formats for optimal performance.
"""

import asyncio
import logging
import time
import sys

from feagi_connector.protocols import ByteStructureID, ProtocolType
from feagi_connector.protocols.serialization import ByteStructureTranslator
from feagi_connector.zmq.client import ZmqFeagiClient

# Configure detailed logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    stream=sys.stdout
)
logger = logging.getLogger("feagi_test")


async def test_connection():
    """Test basic connection to FEAGI."""
    # Create a client
    logger.info("Creating ZMQ client")
    client = ZmqFeagiClient(
        host="localhost",
        control_port=5555,
        sensorimotor_port=5558,
        visualization_port=5560
    )
    
    try:
        # Connect to FEAGI
        logger.info("Connecting to FEAGI")
        connected = await client.connect()
        if not connected:
            logger.error("Failed to connect to FEAGI")
            return
        
        logger.info("Connected to FEAGI successfully")
        
        # Register agent with timeout
        logger.info("Registering agent")
        agent_id = f"test-agent-{int(time.time() * 1000)}"
        agent_type = "test"
        
        try:
            # Try to register with a timeout
            response = await asyncio.wait_for(
                client.register_agent(agent_id, agent_type), 
                timeout=5.0  # 5 second timeout
            )
            logger.info(f"Registration response: {response}")
            
            if response.get("status") == "success":
                logger.info("Agent registered successfully")
                
                # Get status
                logger.info("Getting FEAGI status")
                status = await client.get_status()
                logger.info(f"FEAGI status: {status}")
                
                # Send heartbeat
                logger.info("Sending heartbeat")
                heartbeat = await client.send_heartbeat(agent_id)
                logger.info(f"Heartbeat response: {heartbeat}")
                
                # Wait a bit
                logger.info("Waiting for 2 seconds")
                await asyncio.sleep(2)
                
                # Deregister
                logger.info("Deregistering agent")
                deregister = await client.deregister_agent(agent_id)
                logger.info(f"Deregistration response: {deregister}")
            else:
                logger.error(f"Agent registration failed: {response}")
        except asyncio.TimeoutError:
            logger.error("Registration timed out! FEAGI server might not be responding.")
            
            # Add some diagnostic information
            logger.info("Dumping registration message format for debugging:")
            # Create sample registration message
            request = {
                "type": "register",
                "agent_id": agent_id,
                "agent_type": agent_type,
                "timestamp": int(time.time() * 1000),
                "capabilities": {
                    "protocols": {
                        "fcp": True,
                        "fsmp": True,
                        "fvp": True
                    },
                    "structures": {
                        "1": [1],  # JSON
                        "8": [1],  # RAW_IMAGE
                        "9": [1],  # MULTI_HOLDER
                        "10": [1], # NEURON_FLAT
                        "11": [1]  # NEURON_CATEGORIES
                    }
                }
            }
            
            # Print the request format
            logger.info(f"Request format: {request}")
            
            # Print the byte structure format
            translator = ByteStructureTranslator()
            encoded = translator.encode_message(request)
            logger.info(f"Encoded message (first 20 bytes): {encoded[:20]}")
            logger.info(f"Header: structure_type={encoded[0]}, version={encoded[1]}")
        
    except Exception as e:
        logger.exception(f"Error during test: {e}")
    finally:
        # Disconnect
        logger.info("Disconnecting from FEAGI")
        await client.disconnect()
        logger.info("Test completed")


if __name__ == "__main__":
    asyncio.run(test_connection()) 