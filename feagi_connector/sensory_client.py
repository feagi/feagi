#!/usr/bin/env python3
"""
FEAGI Sensory Client

Client for sending sensory data to FEAGI 2.1 via ZMQ using the PUSH/PULL pattern.
This client properly formats neuron activation data according to FEAGI's expected format.
"""

import sys
import os
import time
import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Union, Set
import random
import json

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq
from feagi_bytes import ByteStructureEncoder, ByteStructureID

# Import our command client
from feagi_client import FeagiClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_sensory_client")


class SensoryClient:
    """Client for sending sensory data to FEAGI on port 5558 using the PUSH socket pattern"""
    
    def __init__(self, host: str = "127.0.0.1", port: int = 5558, cmd_port: int = 5555,
                 reconnect_timeout: float = 5.0, command_timeout: float = 2.0):
        """
        Initialize the sensory client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ sensorimotor port (default 5558)
            cmd_port: ZMQ command port (default 5555)
            reconnect_timeout: Seconds to wait between reconnection attempts
            command_timeout: Timeout for command API calls in seconds
        """
        self.host = host
        self.port = port
        self.cmd_port = cmd_port
        self.agent_id = f"sensory-agent-{random.randint(1000, 9999)}"
        self.context = zmq.Context.instance()
        self.socket = None
        self.reconnect_timeout = reconnect_timeout
        self.command_timeout = command_timeout
        self.last_connection_attempt = 0
        self.connection_healthy = False
        
        # Default cortical area mapping (friendly name -> cortical ID)
        self.cortical_area_map = {
            "visual_input": "i_00CC",
            "audio_input": "i_00CD",
            "touch_input": "i_00CE"
        }
        
        # For checking FEAGI status
        self.cmd_client = FeagiClient(host=host, port=cmd_port)
        self.available_cortical_areas: Set[str] = set()
        self.is_genome_loaded = False
        self.is_simulation_running = False
        self.last_status_update = 0
        self.status_update_interval = 2.0  # seconds
        
    def connect(self, force: bool = False) -> bool:
        """
        Connect to the FEAGI sensorimotor server.
        
        Args:
            force: Force reconnection even if recently attempted
            
        Returns:
            True if connected successfully, False otherwise
        """
        current_time = time.time()
        
        # Don't retry connections too frequently unless forced
        if not force and self.last_connection_attempt > 0:
            if current_time - self.last_connection_attempt < self.reconnect_timeout:
                logger.debug(f"Skipping connection attempt, tried {current_time - self.last_connection_attempt:.1f}s ago")
                return False
        
        # Update connection attempt timestamp
        self.last_connection_attempt = current_time
        self.connection_healthy = False
        
        # Close existing socket if any
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None
        
        try:
            logger.info(f"Connecting to FEAGI sensorimotor server at {self.host}:{self.port}")
            # Use PUSH socket pattern to connect to FEAGI's PULL socket
            self.socket = self.context.socket(zmq.PUSH)
            
            # Configure socket for real-time data
            self.socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
            self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait on close
            self.socket.setsockopt(zmq.RCVTIMEO, int(self.command_timeout * 1000))  # Set timeout
            
            # Connect to the FEAGI server
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            logger.info(f"Connected to FEAGI sensorimotor port")
            
            # Update status information
            update_success = self.update_feagi_status()
            
            # Mark connection as healthy if we could also communicate with the command API
            self.connection_healthy = update_success
            
            return update_success
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            self.connection_healthy = False
            return False
    
    def update_feagi_status(self, force: bool = False) -> bool:
        """
        Update status information from FEAGI.
        
        Args:
            force: Force update even if recently updated
            
        Returns:
            True if status update was successful, False otherwise
        """
        current_time = time.time()
        
        # Don't update status too frequently unless forced
        if not force and self.last_status_update > 0:
            if current_time - self.last_status_update < self.status_update_interval:
                return self.is_simulation_running and self.is_genome_loaded
        
        # Update timestamp
        self.last_status_update = current_time
        
        try:
            # Check if FEAGI is running
            if not self.cmd_client.is_running():
                logger.error("FEAGI is not running")
                self.is_genome_loaded = False
                self.is_simulation_running = False
                self.connection_healthy = False
                return False
            
            # Get current simulation state
            status = self.cmd_client.get_simulation_state()
            self.is_simulation_running = status.get("running", False)
            
            # Get available cortical areas if we don't have them yet
            if not self.available_cortical_areas:
                # Try to get status to see if genome is loaded
                response = self.cmd_client.get_status()
                if "status" in response and "genome_loaded" in response["status"]:
                    self.is_genome_loaded = response["status"]["genome_loaded"]
                else:
                    # Fallback - we'll assume genome is loaded if simulation can be running
                    self.is_genome_loaded = self.is_simulation_running
                
                # If genome is loaded, try to get cortical areas
                if self.is_genome_loaded:
                    genome_response = self.cmd_client.command("get_cortical_areas")
                    if "cortical_areas" in genome_response:
                        self.available_cortical_areas = set(genome_response["cortical_areas"])
                    else:
                        logger.warning("Failed to get cortical areas, will allow sending to any area")
                        # If we can't get the list, we'll assume all areas are valid
                        self.available_cortical_areas = set(self.cortical_area_map.values())
            
            # If we got here, our connection is healthy
            self.connection_healthy = True
            return True
        except Exception as e:
            logger.error(f"Error updating FEAGI status: {e}")
            self.connection_healthy = False
            return False
    
    def get_cortical_id(self, cortical_area: str) -> str:
        """
        Get the cortical ID for a friendly name, or return the input if it looks like an ID already.
        
        Args:
            cortical_area: Friendly name or cortical ID
            
        Returns:
            Cortical ID (i_XXXX format)
        """
        # If it's already in the i_XXXX format, return as is
        if cortical_area.startswith("i_"):
            return cortical_area
            
        # Look up in the mapping
        if cortical_area in self.cortical_area_map:
            return self.cortical_area_map[cortical_area]
            
        # Log a warning and use as is
        logger.warning(f"Unknown cortical area: {cortical_area}. Please use cortical ID directly (i_XXXX format).")
        return cortical_area
    
    def is_cortical_area_available(self, cortical_id: str) -> bool:
        """
        Check if the cortical area is available in the loaded genome.
        
        Args:
            cortical_id: Cortical ID to check
            
        Returns:
            True if the cortical area is available, False otherwise
        """
        # If we couldn't fetch available areas, we'll assume it's available
        if not self.available_cortical_areas:
            return True
        
        return cortical_id in self.available_cortical_areas
    
    def validate_ready_state(self) -> bool:
        """
        Validate FEAGI is in a state ready to receive data.
        
        Returns:
            True if FEAGI is ready, False otherwise
        """
        # First update status
        if not self.update_feagi_status():
            logger.error("Failed to update FEAGI status - FEAGI may be unreachable")
            return False
            
        # Check connection health
        if not self.connection_healthy:
            logger.error("Connection to FEAGI is not healthy")
            return False
            
        # Check if genome is loaded
        if not self.is_genome_loaded:
            logger.error("No genome loaded in FEAGI - sensory data will be ignored")
            return False
            
        # Check if simulation is running (warning only)
        if not self.is_simulation_running:
            logger.warning("Simulation is not running - sensory data may not be processed")
            
        return True
    
    def validate_data(self, neuron_data: Dict[Tuple[int, int, int], float], 
                     cortical_id: str) -> Tuple[bool, str]:
        """
        Validate neuron data before sending.
        
        Args:
            neuron_data: Dictionary mapping neuron coordinates to activation values
            cortical_id: Target cortical ID
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check if neuron data is empty
        if not neuron_data:
            return False, "Neuron data is empty"
            
        # Check if cortical area exists
        if not self.is_cortical_area_available(cortical_id):
            return False, f"Cortical area {cortical_id} is not available in the loaded genome"
            
        # Check activation values are in valid range
        for coords, value in neuron_data.items():
            if not 0 <= value <= 1:
                return False, f"Invalid activation value {value} for neuron {coords} (must be 0-1)"
                
            # Check coordinates are valid (positive integers)
            x, y, z = coords
            if not all(isinstance(c, int) and c >= 0 for c in (x, y, z)):
                return False, f"Invalid coordinates {coords} (must be positive integers)"
                
        return True, ""
    
    def send_neuron_data(self, 
                         cortical_area: str, 
                         neuron_data: Dict[Tuple[int, int, int], float],
                         metadata: Optional[Dict] = None,
                         check_status: bool = True,
                         validate_data: bool = True) -> bool:
        """
        Send neuron activation data to FEAGI.
        
        Args:
            cortical_area: Target cortical area name or ID
            neuron_data: Dictionary mapping neuron coordinates to activation values
            metadata: Optional metadata to include with the data
            check_status: Whether to check FEAGI status before sending
            validate_data: Whether to validate the data before sending
            
        Returns:
            True if sent successfully, False otherwise
        """
        # Ensure we're connected
        if not self.socket:
            if not self.connect():
                return False
        
        try:
            # Get cortical ID
            cortical_id = self.get_cortical_id(cortical_area)
            
            # Check FEAGI status if requested
            if check_status and not self.validate_ready_state():
                return False
            
            # Validate data if requested
            if validate_data:
                is_valid, error_msg = self.validate_data(neuron_data, cortical_id)
                if not is_valid:
                    logger.error(f"Data validation failed: {error_msg}")
                    return False
            
            # Convert coordinate dict to lists for encoding
            cortical_ids = []
            x_coords = []
            y_coords = []
            z_coords = []
            potentials = []
            
            for (x, y, z), value in neuron_data.items():
                cortical_ids.append(cortical_id)  # Use the proper cortical ID
                x_coords.append(x)
                y_coords.append(y)
                z_coords.append(z)
                potentials.append(value)
            
            logger.debug(f"Encoding {len(cortical_ids)} neurons for {cortical_id}")
            
            # Encode the data using encode_neuron_flat with proper parameters
            encoder = ByteStructureEncoder()
            
            # Include metadata if provided
            if metadata:
                # Update metadata to use cortical_id instead of cortical_area
                if "cortical_area" in metadata:
                    metadata["cortical_id"] = cortical_id
                    del metadata["cortical_area"]
                elif "cortical_id" not in metadata:
                    metadata["cortical_id"] = cortical_id
                
                metadata_bytes = encoder.encode_json(
                    data=metadata
                )
                
                # Send metadata first
                self.socket.send(metadata_bytes, zmq.SNDMORE)
            
            # Encode and send the neuron data
            neuron_bytes = encoder.encode_neuron_flat(
                cortical_ids=cortical_ids,
                x_coords=x_coords,
                y_coords=y_coords,
                z_coords=z_coords,
                potentials=potentials
            )
            
            # Send the neuron data
            self.socket.send(neuron_bytes)
            
            logger.info(f"Sent {len(neuron_data)} neurons to {cortical_id}")
            return True
            
        except zmq.error.Again as e:
            logger.error(f"Timeout sending data: {e}")
            # Mark connection as unhealthy
            self.connection_healthy = False
            return False
        except Exception as e:
            logger.error(f"Error sending neuron data: {e}")
            # Mark connection as unhealthy on any error
            self.connection_healthy = False
            return False
    
    def generate_test_data(self, 
                           cortical_area: str,
                           width: int = 10, 
                           height: int = 10, 
                           depth: int = 1, 
                           activation_rate: float = 0.3) -> Dict[Tuple[int, int, int], float]:
        """
        Generate random test data for the specified cortical area.
        
        Args:
            cortical_area: Target cortical area or ID
            width: Width of the generated pattern
            height: Height of the generated pattern
            depth: Depth of the generated pattern
            activation_rate: Percentage of neurons to activate (0.0-1.0)
            
        Returns:
            Dictionary mapping neuron coordinates to activation values
        """
        data = {}
        
        for x in range(width):
            for y in range(height):
                for z in range(depth):
                    # Only activate some neurons randomly based on activation_rate
                    if random.random() < activation_rate:
                        # Generate activation value between 0.1 and 1.0
                        data[(x, y, z)] = 0.1 + random.random() * 0.9
        
        return data
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
        logger.info("Sensory client closed")


def test_client():
    """Run a simple test of the sensory client"""
    client = SensoryClient()
    
    try:
        logger.info("Testing sensory client...")
        
        # Check if FEAGI is ready
        if not client.validate_ready_state():
            logger.warning("FEAGI is not in a ready state - test may not work properly")
        
        # Send a few test patterns
        for i in range(5):
            logger.info(f"Sending test data iteration {i+1}/5")
            
            # Generate random test data
            test_data = client.generate_test_data(
                cortical_area="visual_input",
                width=10,
                height=10,
                activation_rate=0.2
            )
            
            # Add metadata
            metadata = {
                "agent_id": client.agent_id,
                "timestamp": time.time(),
                "message_type": "sensory_data",
                "cortical_id": client.get_cortical_id("visual_input")  # Use cortical_id
            }
            
            # Send the data
            if client.send_neuron_data("visual_input", test_data, metadata):
                logger.info(f"Successfully sent test data {i+1}/5")
            else:
                logger.error(f"Failed to send test data {i+1}/5")
                # Try to reconnect for the next iteration
                client.connect(force=True)
            
            time.sleep(1)
            
    finally:
        client.close()


if __name__ == "__main__":
    test_client() 