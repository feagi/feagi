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
FEAGI Client

Main client interface for connecting to FEAGI and exchanging data through four stream types:
1. Control Stream: Bidirectional JSON-based messaging for control commands and health status
2. Sensory Stream: One-directional binary data flow from agent to FEAGI
3. Motor Stream: One-directional binary data flow from FEAGI to agent
4. Visualization Stream: One-directional binary data flow from FEAGI to agent for neuron activity
"""

import asyncio
import json
import logging
import uuid
import time
from typing import Dict, Any, Optional, List, Union, Tuple, Callable, Awaitable

import zmq
import zmq.asyncio
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureID

from feagi_connector.api.command_client import FeagiControlClient
from feagi_connector.api.sensory_client import FeagiSensoryClient
from feagi_connector.api.motor_client import FeagiMotorClient
from feagi_connector.api.viz_client import FeagiVizClient
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ
from feagi_connector.utils.processing import (
    infer_byte_structure_type_python as infer_byte_structure_type,
    extract_sub_structures_python as extract_sub_structures
)

# Configure logging
logger = logging.getLogger("feagi_connector")


class FeagiClient:
    """
    High-level client for connecting to FEAGI.
    
    This client provides a unified interface for all FEAGI communication streams:
    1. Control Stream: Bidirectional JSON-based messaging for control commands and health status
    2. Sensory Stream: One-directional binary data flow from agent to FEAGI
    3. Motor Stream: One-directional binary data flow from FEAGI to agent
    4. Visualization Stream: One-directional binary data flow from FEAGI to agent for neuron activity
    
    It handles connection management, agent registration, and data exchange.
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        control_port: int = 5555,
        rest_port: int = 5563,
        sensory_port: int = 5558,
        motor_port: int = 5564,
        visualization_port: int = 5560,
        agent_id: Optional[str] = None,
        agent_type: str = "external",
        timeout: int = 5000,
    ):
        """
        Initialize the FEAGI client.
        
        Args:
            host: FEAGI hostname or IP
            control_port: Control stream port (default 5555)
            rest_port: REST Stream API port (default 5563)
            sensory_port: Sensory stream port (default 5558)
            motor_port: Motor stream port (default 5564)
            visualization_port: Visualization stream port (default 5560)
            agent_id: Agent ID for FEAGI registration (default: auto-generated)
            agent_type: Agent type for categorization
            timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.agent_id = agent_id or f"agent-{uuid.uuid4().hex[:8]}"
        self.agent_type = agent_type
        self.timeout = timeout
        
        # Initialize ZMQ context
        self.context = zmq.asyncio.Context.instance()
        
        # Initialize individual clients for different FEAGI streams
        self.command_client = FeagiControlClient(
            host=host, 
            port=control_port,
            timeout=timeout
        )
        
        # REST client for agent registration
        self.rest_client = FeagiControlClient(host=host, port=rest_port, timeout=timeout)
        
        self.sensory_client = FeagiSensoryClient(
            host=host,
            port=sensory_port,
            timeout=timeout
        )
        
        self.motor_client = FeagiMotorClient(
            host=host,
            port=motor_port,
            agent_id=agent_id,
            socket_timeout=timeout
        )
        
        self.viz_client = FeagiVizClient(
            host=host,
            port=visualization_port,
            agent_id=agent_id,
            socket_timeout=timeout
        )
        
        # State
        self.connected = False
        self.registered = False
        self.motor_callback = None
        self.visualization_callback = None
        
        # Tasks
        self.heartbeat_task = None
        self.motor_listen_task = None
        self.viz_listen_task = None
    
    async def register_with_capabilities(
        self,
        capabilities_file: str = None,
        capabilities_data: Dict[str, Any] = None
    ) -> bool:
        """
        Register agent with FEAGI using capabilities from file or data.
        
        Args:
            capabilities_file: Path to capabilities.json file
            capabilities_data: Capabilities data dictionary (alternative to file)
            
        Returns:
            True if registration was successful
        """
        import json
        import os
        
        # Load capabilities
        if capabilities_data:
            full_capabilities = capabilities_data
        elif capabilities_file and os.path.exists(capabilities_file):
            try:
                with open(capabilities_file, 'r') as f:
                    full_capabilities = json.load(f)
            except Exception as e:
                logger.error(f"Failed to load capabilities file {capabilities_file}: {e}")
                return False
        else:
            logger.warning("No capabilities file or data provided, using defaults")
            full_capabilities = None
        
        # Extract simple capabilities from full structure
        simple_capabilities = {
            "sensory": False,
            "motor": False,
            "visualization": bool(self.visualization_callback)
        }
        
        if full_capabilities and "capabilities" in full_capabilities:
            caps = full_capabilities["capabilities"]
            simple_capabilities["sensory"] = bool(caps.get("input", {}))
            simple_capabilities["motor"] = bool(caps.get("output", {}))
        
        # Register via REST Stream API
        registration_result = await self.rest_client.register_agent(
            agent_id=self.agent_id,
            agent_type=self.agent_type,
            capabilities=simple_capabilities,
            full_capabilities=full_capabilities
        )
        
        if "error" in registration_result:
            logger.error(f"Failed to register agent: {registration_result['error']}")
            return False
        
        # Check if registration was successful
        response_status = registration_result.get("status", 500)
        if response_status == 200:
            response_body = registration_result.get("body", {})
            logger.info(f"✅ Agent registered successfully: {response_body.get('message', 'OK')}")
            
            # Log FQ sampler coordination info if available
            fq_info = response_body.get("fq_samplers_enabled", {})
            if fq_info:
                logger.info(f"🔄 FQ Samplers enabled: {fq_info}")
            
            self.registered = True
            return True
        else:
            logger.error(f"❌ Agent registration failed with status {response_status}: {registration_result}")
            return False

    async def connect(self) -> bool:
        """Connect to FEAGI with proper registration."""
        try:
            # Step 1: Connect all clients
            if not await self.command_client.connect():
                logger.error("Failed to connect to FEAGI command stream")
                return False
                
            if not self.sensory_client.connect():
                logger.error("Failed to connect to FEAGI sensory stream")
                return False
                
            # Step 2: Register agent if not already registered
            if not self.registered:
                try:
                    success = await self.rest_client.register_agent(self.agent_id, self.agent_type)
                    if success:
                        self.registered = True
                        logger.info("✅ Agent registered successfully")
                    else:
                        logger.warning("⚠️ Agent registration failed, but continuing...")
                except Exception as e:
                    logger.warning(f"⚠️ Agent registration error: {e}, but continuing...")
            
            # Step 3: Start motor and visualization listeners ONLY if callbacks are registered
            if self.motor_callback and hasattr(self.motor_client, 'register_motor_callback'):
                self.motor_client.register_motor_callback(self.motor_callback)
                if hasattr(self.motor_client, 'start'):
                    await self.motor_client.start()
                logger.info("✅ Motor client started with callback")
            else:
                logger.info("ℹ️ Motor client not started (no callback registered)")
            
            self.connected = True
            logger.info("✅ Successfully connected to FEAGI")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to FEAGI: {e}")
            return False
    
    async def disconnect(self) -> None:
        """
        Disconnect from FEAGI.
        """
        self.connected = False
        
        # Send goodbye message
        try:
            await self.command_client.send_goodbye(agent_id=self.agent_id, agent_type=self.agent_type)
        except Exception as e:
            logger.warning(f"Error sending goodbye message: {e}")
        
        # Cancel background tasks
        if self.heartbeat_task:
            self.heartbeat_task.cancel()
            try:
                await self.heartbeat_task
            except asyncio.CancelledError:
                pass
            self.heartbeat_task = None
        
        if self.motor_listen_task:
            self.motor_listen_task.cancel()
            try:
                await self.motor_listen_task
            except asyncio.CancelledError:
                pass
            self.motor_listen_task = None
        
        if self.viz_listen_task:
            self.viz_listen_task.cancel()
            try:
                await self.viz_listen_task
            except asyncio.CancelledError:
                pass
            self.viz_listen_task = None
        
        # Close clients
        await self.command_client.close()
        self.sensory_client.close()  # This is now a synchronous method
        await self.motor_client.close()
        await self.viz_client.close()
        
        logger.info("Disconnected from FEAGI")
    
    def send_sensory_data(
        self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]
    ) -> bool:
        """Send sensory data to FEAGI (synchronous)."""
        if not self.connected:
            logger.error("Not connected to FEAGI")
            return False
            
        return self.sensory_client.send_sensory_data(cortical_area, neuron_data)
    
    def register_motor_callback(
        self, 
        callback: Callable[[str, bytes], None]
    ) -> None:
        """
        Register a callback for motor data.
        
        Args:
            callback: Function to call when motor data is received
                     (parameters: channel_id, data)
        """
        self.motor_callback = callback
        if self.connected and not self.motor_listen_task:
            self.motor_listen_task = asyncio.create_task(self._motor_listen_loop())
    
    def register_visualization_callback(
        self,
        callback: Callable[[bytes], None]
    ) -> None:
        """
        Register a callback for visualization data.
        
        Args:
            callback: Function to call when visualization data is received
                     (parameter: data)
        """
        logger.info(f"Registering visualization callback function: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous'}")
        self.visualization_callback = callback
        if self.connected and not self.viz_listen_task:
            logger.debug("Already connected, starting visualization listener task")
            self.viz_listen_task = asyncio.create_task(self._viz_listen_loop())
        elif not self.connected:
            logger.debug("Not connected yet, visualization listener will start on connection")
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get FEAGI system status.
        
        Returns:
            Dictionary with system status information
        """
        if not self.connected:
            logger.error("Not connected to FEAGI")
            return {"error": "Not connected"}
        
        return await self.command_client.get_status()
    
    async def _heartbeat_loop(self) -> None:
        """Send periodic heartbeats to keep the connection alive."""
        try:
            while self.connected:
                await self.command_client.make_request("heartbeat", {"agent_id": self.agent_id, "agent_type": self.agent_type})
                await asyncio.sleep(30)  # Send heartbeat every 30 seconds
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in heartbeat loop: {e}")
            self.connected = False
    
    async def _motor_listen_loop(self) -> None:
        """Listen for motor data."""
        try:
            # Register callback and start motor client
            self.motor_client.register_motor_callback(self.motor_callback)
            await self.motor_client.start()
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in motor listen loop: {e}")
            self.connected = False
    
    async def _viz_listen_loop(self) -> None:
        """Listen for visualization data."""
        logger.info("Starting visualization listen loop")
        try:
            start_time = time.time()
            logger.debug(f"Calling viz_client.start_visualization_listener with callback: {self.visualization_callback.__name__ if hasattr(self.visualization_callback, '__name__') else 'anonymous'}")
            await self.viz_client.start_visualization_listener(self.visualization_callback)
            total_time = time.time() - start_time
            logger.info(f"Visualization listener exited after {total_time:.1f} seconds")
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            logger.info("Visualization listen loop cancelled")
            pass
        except Exception as e:
            logger.error(f"Error in visualization listen loop: {e}", exc_info=True)
            self.connected = False
        logger.debug("Visualization listen loop exited") 