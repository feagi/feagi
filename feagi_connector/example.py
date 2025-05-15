#!/usr/bin/env python
"""
FEAGI Connector Example

This example demonstrates how to use the FEAGI connector to interact with
a running FEAGI instance using the byte structure protocol.
"""

import asyncio
import logging
import time
from typing import List, Dict, Any

from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel, ByteStructureID

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_example")


class SimpleFeagiAgent:
    """
    Simple example agent that connects to FEAGI.
    
    This agent sends simulated sensory data and processes motor responses.
    """
    
    def __init__(self, host: str = "localhost", agent_id: str = None):
        """
        Initialize the agent.
        
        Args:
            host: FEAGI host address
            agent_id: Optional agent identifier (auto-generated if not provided)
        """
        self.client = FeagiClient(
            host=host,
            agent_id=agent_id,
            agent_type="example"
        )
        self.running = False
        self.received_motor_data = []
        self.last_neural_activity = None
    
    async def start(self):
        """Start the agent."""
        # Connect to FEAGI
        connected = await self.client.connect()
        if not connected:
            logger.error("Failed to connect to FEAGI")
            return False
        
        # Register callbacks for receiving data
        await self.client.register_motor_callback(self._handle_motor_data)
        await self.client.register_visualization_callbacks(
            activity_callback=self._handle_activity_data,
            structure_callback=self._handle_structure_data
        )
        
        self.running = True
        logger.info("Agent started")
        return True
    
    async def stop(self):
        """Stop the agent."""
        if self.running:
            await self.client.disconnect()
            self.running = False
            logger.info("Agent stopped")
    
    def _handle_motor_data(self, channel_id: int, data: bytes):
        """Handle incoming motor data from FEAGI."""
        logger.info(f"Received motor data on channel {channel_id}: {len(data)} bytes")
        self.received_motor_data.append((channel_id, data))
    
    def _handle_activity_data(self, data: bytes):
        """Handle incoming neural activity data from FEAGI."""
        # The data is now a byte structure containing neuron potentials
        structure_type, version = data[0], data[1]  # First two bytes are header
        logger.info(f"Received neural activity data: {len(data)} bytes, type={structure_type}, version={version}")
        self.last_neural_activity = data
    
    def _handle_structure_data(self, data: bytes):
        """Handle incoming brain structure data from FEAGI."""
        # The data is now a byte structure containing brain structure
        structure_type, version = data[0], data[1]  # First two bytes are header
        logger.info(f"Received brain structure data: {len(data)} bytes, type={structure_type}, version={version}")
    
    async def send_dummy_sensory_data(self):
        """Send dummy sensory data to FEAGI."""
        if not self.running:
            return
        
        # Create dummy data (e.g., a small image)
        width, height = 10, 10
        data = bytes([0x80] * (width * height))  # 10x10 grayscale image
        
        # Send to vision channel
        await self.client.send_sensory_data(channel_id=FSMPChannel.VISION, data=data)
        logger.info(f"Sent dummy sensory data: {width}x{height} image")
    
    async def heartbeat_loop(self):
        """Periodically send heartbeats to maintain connection."""
        while self.running:
            try:
                response = await self.client.send_heartbeat(self.client.agent_id)
                if response.get("type") == "heartbeat_response":
                    logger.debug("Heartbeat acknowledged")
                else:
                    logger.warning(f"Unexpected heartbeat response: {response}")
            except Exception as e:
                logger.error(f"Error sending heartbeat: {e}")
                
            await asyncio.sleep(5)  # Send heartbeat every 5 seconds


async def run_example():
    """Run the example agent."""
    agent = SimpleFeagiAgent(host="localhost")
    
    try:
        # Start the agent
        started = await agent.start()
        if not started:
            logger.error("Failed to start agent")
            return
        
        # Start heartbeat loop in background
        heartbeat_task = asyncio.create_task(agent.heartbeat_loop())
        
        # Send some sensory data
        for _ in range(5):
            await agent.send_dummy_sensory_data()
            await asyncio.sleep(1)
        
        # Get status from FEAGI
        status = await agent.client.get_status()
        logger.info(f"FEAGI status: {status}")
        
        # Keep running for a bit to receive responses
        logger.info("Waiting for motor responses...")
        await asyncio.sleep(5)
        
    finally:
        # Stop the agent
        await agent.stop()
        
        # Cancel heartbeat task
        if 'heartbeat_task' in locals():
            heartbeat_task.cancel()
            try:
                await heartbeat_task
            except asyncio.CancelledError:
                pass


if __name__ == "__main__":
    asyncio.run(run_example()) 