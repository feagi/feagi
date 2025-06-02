#!/usr/bin/env python3
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
FEAGI Example Agent

This example demonstrates how to create an agent that connects to FEAGI
and exchanges sensory and motor data.
"""

import asyncio
import logging
import numpy as np
from typing import Dict, Tuple, Any, Optional

from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel
from feagi_connector.utils import setup_logging


class ExampleAgent:
    """
    Example agent that connects to FEAGI and exchanges data.
    
    This agent:
    1. Connects to FEAGI
    2. Sends simulated sensory data
    3. Receives and processes motor data
    4. Visualizes neural activity
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        agent_id: Optional[str] = None,
        log_level: int = logging.INFO
    ):
        """
        Initialize the example agent.
        
        Args:
            host: FEAGI hostname or IP
            agent_id: Agent ID (default: auto-generated)
            log_level: Logging level
        """
        # Set up logging
        setup_logging(level=log_level, log_file="example_agent.log")
        self.logger = logging.getLogger("example_agent")
        
        # Create FEAGI client
        self.client = FeagiClient(
            host=host,
            agent_id=agent_id or "example-agent",
            agent_type="example"
        )
        
        # State
        self.running = False
        self.image_size = (10, 10)  # 10x10 image
        
    async def connect(self) -> bool:
        """
        Connect to FEAGI.
        
        Returns:
            True if connection was successful
        """
        self.logger.info("Connecting to FEAGI...")
        
        # Connect to FEAGI
        connected = await self.client.connect()
        if not connected:
            self.logger.error("Failed to connect to FEAGI")
            return False
        
        # Register callbacks
        await self.client.register_motor_callback(self._handle_motor_data)
        
        # Register visualization callbacks
        await self.client.register_visualization_callbacks(
            activity_callback=self._handle_activity_data,
            structure_callback=self._handle_structure_data
        )
        
        self.logger.info("Connected to FEAGI")
        return True
    
    async def disconnect(self) -> None:
        """Disconnect from FEAGI."""
        self.running = False
        await self.client.disconnect()
        self.logger.info("Disconnected from FEAGI")
    
    async def run(self, duration_seconds: int = 60) -> None:
        """
        Run the agent for the specified duration.
        
        Args:
            duration_seconds: Duration to run in seconds
        """
        if not await self.connect():
            return
        
        self.running = True
        self.logger.info(f"Running agent for {duration_seconds} seconds")
        
        try:
            # Get FEAGI status
            status = await self.client.get_status()
            self.logger.info(f"FEAGI status: {status}")
            
            # Request visualization data
            await self.client.request_visualization()
            
            # Run for the specified duration
            start_time = asyncio.get_event_loop().time()
            while self.running and (asyncio.get_event_loop().time() - start_time) < duration_seconds:
                # Generate and send sensory data
                await self._send_sensory_data()
                
                # Request visualization data periodically
                if int(asyncio.get_event_loop().time()) % 5 == 0:  # Every 5 seconds
                    await self.client.request_visualization()
                
                # Sleep for a bit
                await asyncio.sleep(0.1)
                
        except asyncio.CancelledError:
            self.logger.info("Agent run cancelled")
        except Exception as e:
            self.logger.error(f"Error during agent run: {e}")
        finally:
            await self.disconnect()
    
    async def _send_sensory_data(self) -> None:
        """Generate and send simulated sensory data to FEAGI."""
        # Generate a simple pattern: a moving dot
        t = asyncio.get_event_loop().time()
        x = int((t % 10) * 0.5) % self.image_size[0]
        y = int((t % 10) * 0.3) % self.image_size[1]
        
        # Create a 10x10 image with a single active pixel
        image_data = np.zeros(self.image_size, dtype=np.uint8)
        image_data[y, x] = 255
        
        # Convert to bytes
        binary_data = image_data.tobytes()
        
        # Send to FEAGI
        self.logger.debug(f"Sending sensory data: dot at ({x}, {y})")
        await self.client.send_sensory_data(FSMPChannel.VISION, binary_data)
    
    def _handle_motor_data(self, channel_id: int, data: bytes) -> None:
        """
        Handle motor data from FEAGI.
        
        Args:
            channel_id: Motor channel ID
            data: Motor data bytes
        """
        self.logger.info(f"Received motor data on channel {channel_id}: {len(data)} bytes")
        
        # Process based on channel
        if channel_id == FSMPChannel.MOTOR_ARM.value:
            self.logger.info("Arm movement command received")
            # Process arm movement...
        elif channel_id == FSMPChannel.MOTOR_SPEECH.value:
            self.logger.info("Speech command received")
            # Process speech...
    
    def _handle_activity_data(self, data: Dict[str, Any]) -> None:
        """
        Handle neural activity data from FEAGI.
        
        Args:
            data: Neural activity data
        """
        activity = data.get("data", {})
        neuron_count = len(activity)
        self.logger.info(f"Received neural activity data: {neuron_count} active neurons")
        
        # Process neural activity...
        # In a real application, this might update a visualization
    
    def _handle_structure_data(self, data: Dict[str, Any]) -> None:
        """
        Handle brain structure data from FEAGI.
        
        Args:
            data: Brain structure data
        """
        self.logger.info("Received brain structure data")
        
        # Process brain structure...
        # In a real application, this might update a 3D model


async def main():
    """Main entry point."""
    # Create and run the example agent
    agent = ExampleAgent(host="127.0.0.1")
    await agent.run(duration_seconds=60)


if __name__ == "__main__":
    asyncio.run(main()) 