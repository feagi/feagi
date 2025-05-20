"""
ZMQ REST Client for FEAGI Connector

This module extends the FEAGI connector with support for the ZMQ REST API protocol,
allowing use of the same REST API that's available over HTTP, but via ZMQ transport.
It adds capabilities for loading genomes and other REST API operations.
"""

import json
import logging
import os
import sys
import time
from typing import Dict, Any, Optional, List, Union

# Use our local implementation of ZMQRestClient
from feagi_connector import FeagiClient
from feagi_connector.zmq.rest_client import ZMQRestClient

logger = logging.getLogger("feagi_connector.zmq_rest")

class FeagiZmqRestClient(FeagiClient):
    """
    FEAGI client with ZMQ REST API support.
    
    This class extends the standard FEAGI client with support for
    using the ZMQ REST API protocol for operations like loading genomes.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        agent_id: str = None,
        agent_type: str = "rest_client",
        zmq_rest_port: int = 5560,
        **kwargs
    ):
        """
        Initialize the ZMQ REST FEAGI client.
        
        Args:
            host: FEAGI host address
            agent_id: Unique identifier for this agent (generated if not provided)
            agent_type: Type of agent (for categorization)
            zmq_rest_port: Port for ZMQ REST API
            **kwargs: Additional transport-specific parameters
        """
        super().__init__(host=host, agent_id=agent_id, agent_type=agent_type, **kwargs)
        
        # Create the ZMQ REST client
        self.rest_client = ZMQRestClient(
            host=host,
            port=zmq_rest_port,
            timeout=30
        )
        
    async def connect(self) -> bool:
        """
        Connect to FEAGI and register the agent.
        
        This method connects both the standard ZMQ channels and the ZMQ REST API.
        
        Returns:
            True if connected successfully, False otherwise
        """
        # Connect the standard client
        connected = await super().connect()
        if not connected:
            return False
            
        # Connect the REST client
        try:
            self.rest_client.connect()
            logger.info("Connected to FEAGI ZMQ REST API")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to FEAGI ZMQ REST API: {e}")
            await super().disconnect()
            return False
            
    async def disconnect(self) -> bool:
        """
        Disconnect from FEAGI.
        
        Returns:
            True if disconnected successfully, False otherwise
        """
        # Disconnect the REST client
        try:
            self.rest_client.disconnect()
        except Exception as e:
            logger.warning(f"Error disconnecting from ZMQ REST API: {e}")
            
        # Disconnect the standard client
        return await super().disconnect()
        
    async def load_genome(self, genome_path: Optional[str] = None) -> bool:
        """
        Load a genome into FEAGI using the ZMQ REST API.
        
        Args:
            genome_path: Path to genome JSON file, defaults to test_genome.json in the package
            
        Returns:
            True if the genome was loaded successfully, False otherwise
        """
        try:
            # Use default test genome if no path provided
            if genome_path is None:
                genome_path = os.path.join(os.path.dirname(__file__), "test_genome.json")
                
            logger.info(f"Loading genome from {genome_path}")
            
            # Read the genome file
            with open(genome_path, 'r') as f:
                genome_data = json.load(f)
                
            # Send the genome to FEAGI using the REST API
            logger.info(f"Uploading genome to FEAGI ({len(json.dumps(genome_data))} bytes)")
            result = self.rest_client.post('/v1/genome/upload', body=genome_data)
            
            if 'message' in result and 'success' in result.get('message', '').lower():
                logger.info("Genome loaded successfully")
                return True
            else:
                logger.error(f"Failed to load genome: {result}")
                return False
                
        except Exception as e:
            logger.exception(f"Error loading genome: {e}")
            return False
            
    async def get_available_cortical_areas(self) -> List[Dict[str, Any]]:
        """
        Get list of available cortical areas from FEAGI.
        
        Returns:
            List of cortical area dictionaries
        """
        try:
            return self.rest_client.get_cortical_areas()
        except Exception as e:
            logger.error(f"Error getting cortical areas: {e}")
            return []
            
    async def send_sensory_data_to_cortical_area(self, cortical_area_id: str, data: List[List[int]]) -> bool:
        """
        Send sensory data directly to a cortical area using the REST API.
        
        Args:
            cortical_area_id: ID of the target cortical area
            data: List of [x, y, z] coordinates to activate
            
        Returns:
            True if sent successfully, False otherwise
        """
        try:
            # Format the data for the direct stimulation API
            stim_data = {
                "coordinates": data
            }
            
            # Send to FEAGI
            result = self.rest_client.post(
                f'/v1/connectome/cortical_area/{cortical_area_id}/stimulate',
                body=stim_data,
                params={'cortical_id': cortical_area_id}
            )
            
            logger.info(f"Sent {len(data)} activations to {cortical_area_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False 