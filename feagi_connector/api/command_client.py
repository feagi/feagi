"""
FEAGI Control Client

Client for the FEAGI control API using ZMQ REQ/REP pattern.
"""

import json
import logging
import time
from typing import Dict, Any, Optional, List, Union

import zmq
import zmq.asyncio

# Configure logging
logger = logging.getLogger("feagi_connector.control")


class FeagiControlClient:
    """
    Control client for FEAGI using REQ/REP pattern (port 5559).
    
    This client implements the control-based API over ZMQ REQ/REP pattern,
    correctly handling message formatting and state.
    """
    
    def __init__(self, host: str = "127.0.0.1", port: int = 5559, timeout: int = 5000, auth_token: Optional[str] = None):
        """
        Initialize the control client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ REQ/REP port (default 5559)
            timeout: Request timeout in milliseconds
            auth_token: Authentication token (optional)
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.auth_token = auth_token
        self.context = zmq.asyncio.Context.instance()
        self.next_id = 1
    
    async def make_request(self, command: str, params: Optional[Dict] = None) -> Dict:
        """
        Make a direct command request with proper message formatting.
        
        Args:
            command: Command name (e.g., 'ping', 'get_status')
            params: Command parameters
            
        Returns:
            Response data or error dictionary
        """
        # Create a new socket for this request (important for REQ/REP state)
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        try:
            # Connect to the server
            socket.connect(f"tcp://{self.host}:{self.port}")
            logger.debug(f"Connected to {self.host}:{self.port}")
            
            # Create command request
            request = {
                'command': command,
                'id': self.next_id,
                'timestamp': int(time.time() * 1000),
                'params': params or {}
            }
            self.next_id += 1
                
            # Serialize request data
            request_bytes = json.dumps(request).encode('utf-8')
            content_type = "application/json"
            auth_token_bytes = self.auth_token.encode('utf-8') if self.auth_token else b""
            
            logger.debug(f"Sending command: {command} ({len(request_bytes)} bytes)")
            
            # Send request with correct format: [auth_token, content_type, request_data]
            await socket.send_multipart([
                auth_token_bytes,              # Auth token (empty if none)
                content_type.encode('utf-8'),  # Content type
                request_bytes                  # Request data
            ])
            
            # Wait for response
            try:
                response_parts = await socket.recv_multipart()
                
                # FEAGI should respond with [content_type, response_data]
                if len(response_parts) < 2:
                    logger.warning(f"Unexpected response format - received {len(response_parts)} parts")
                    return {'error': 'Invalid response format'}
                
                # Parse the response
                resp_content_type = response_parts[0].decode('utf-8')
                resp_data = response_parts[1]
                
                logger.debug(f"Received response: {len(resp_data)} bytes, content-type: {resp_content_type}")
                
                if resp_content_type == "application/json":
                    try:
                        # Parse JSON response
                        response = json.loads(resp_data.decode('utf-8'))
                        return response
                    except json.JSONDecodeError:
                        logger.warning(f"Could not decode JSON response")
                        return {'raw_response': resp_data.decode('utf-8', errors='replace')}
                else:
                    # Return raw response with content type
                    return {
                        'content_type': resp_content_type,
                        'raw_response': resp_data.decode('utf-8', errors='replace')
                    }
                
            except zmq.error.Again:
                logger.error(f"Request timed out after {self.timeout/1000} seconds")
                return {'error': 'timeout'}
                
        except Exception as e:
            logger.error(f"Error: {e}")
            return {'error': 'general', 'message': str(e)}
            
        finally:
            # Always close the socket to prevent resource leaks
            socket.close()
    
    # Convenience methods for common commands
    async def ping(self) -> Dict:
        """Send a ping command."""
        return await self.make_request("ping")
    
    async def get_status(self) -> Dict:
        """Get the simulation status."""
        return await self.make_request("get_status")
    
    async def get_configuration(self) -> Dict:
        """Get the FEAGI configuration."""
        return await self.make_request("get_configuration")
    
    async def get_performance(self) -> Dict:
        """Get performance metrics."""
        return await self.make_request("get_performance")
    
    async def get_cortical_areas(self) -> Dict:
        """Get cortical areas."""
        return await self.make_request("get_cortical_areas")
    
    async def start_simulation(self) -> Dict:
        """Start the simulation."""
        return await self.make_request("start_simulation")
    
    async def stop_simulation(self) -> Dict:
        """Stop the simulation."""
        return await self.make_request("stop_simulation")
    
    async def list_genomes(self) -> Dict:
        """List available genomes."""
        return await self.make_request("list_genomes")
    
    async def load_genome(self, name: str) -> Dict:
        """
        Load a genome.
        
        Args:
            name: Name of the genome to load
            
        Returns:
            Response dictionary
        """
        return await self.make_request("load_genome", {"name": name}) 