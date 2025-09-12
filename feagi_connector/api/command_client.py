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
    Control client for FEAGI using REQ/REP pattern (port 5555).
    
    This client implements the control-based API over ZMQ REQ/REP pattern,
    correctly handling message formatting and state.
    """
    
    def __init__(self, host: str = "127.0.0.1", port: int = 5555, timeout: int = 5000, auth_token: Optional[str] = None):
        """
        Initialize the control client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ REQ/REP port (default 5555)
            timeout: Request timeout in milliseconds
            auth_token: Authentication token (optional)
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.auth_token = auth_token
        self.context = zmq.asyncio.Context.instance()
        self.next_id = 1
    
    async def connect(self) -> bool:
        """
        Connect to the FEAGI control stream.
        
        This method doesn't actually establish a persistent connection since 
        each request creates a new socket, but it's provided for API consistency.
        
        Returns:
            True indicating success (always returns True)
        """
        # Test connection by sending a ping
        try:
            result = await self.ping()
            if 'error' in result:
                logger.error(f"Failed to connect to control stream: {result['error']}")
                return False
            # Check for successful ping response (FEAGI returns {"pong": True, ...})
            if result.get('pong') is True:
                return True
            logger.error(f"Unexpected ping response: {result}")
            return False
        except Exception as e:
            logger.error(f"Error connecting to control stream: {e}")
            return False
    
    async def close(self) -> None:
        """
        Clean up resources.
        
        This method doesn't need to do much since each request creates and closes its own socket.
        """
        # Nothing to do here, each request creates and closes its own socket
        pass
    
    async def register_agent(
        self,
        agent_id: str,
        agent_type: str = "external",
        capabilities: Optional[Dict[str, Any]] = None,
        full_capabilities: Optional[Dict[str, Any]] = None,
        agent_version: Optional[str] = None,
        agent_ip: Optional[str] = None
    ) -> Dict:
        """
        Register agent with FEAGI using REST Stream format.
        
        Args:
            agent_id: Unique agent identifier
            agent_type: Type of agent (external, brain_visualizer, etc.)
            capabilities: Simple boolean capabilities (sensory, motor, visualization, etc.)
            full_capabilities: Full capabilities structure from capabilities.json
            agent_version: Agent version string
            agent_ip: Agent IP address
            
        Returns:
            Registration response data
        """
        # Use REST Stream format as documented in FEAGI
        rest_message = {
            "route": "/v1/agent/register",
            "method": "POST",
            "body": {
                "agent_id": agent_id,
                "agent_type": agent_type,
                "capabilities": capabilities or {
                    "sensory": True,
                    "motor": True,
                    "visualization": False
                },
                "agent_version": agent_version or "1.0.0",
                "controller_version": "2.0.0",
                "agent_data_port": 0,  # 0 indicates no specific port requirement
                "agent_ip": agent_ip or "127.0.0.1"
            },
            "timestamp": int(time.time() * 1000)
        }
        
        # Include full capabilities if provided
        if full_capabilities:
            rest_message["body"]["full_capabilities"] = full_capabilities
        
        return await self.make_rest_request(rest_message)

    async def make_rest_request(self, rest_message: Dict) -> Dict:
        """
        Make a REST Stream request to FEAGI.
        
        Args:
            rest_message: REST Stream format message
            
        Returns:
            Response data or error dictionary
        """
        socket = None
        try:
            socket = self.context.socket(zmq.REQ)
            socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            socket.setsockopt(zmq.LINGER, 0)
            socket.connect(f"tcp://{self.host}:{self.port}")
            
            logger.debug(f"Sending REST request to {self.host}:{self.port}")
            logger.debug(f"REST message: {json.dumps(rest_message, indent=2)}")
            
            # Send REST Stream format message
            await socket.send_string(json.dumps(rest_message))
            
            # Receive response
            response_str = await socket.recv_string()
            response = json.loads(response_str)
            
            logger.debug(f"Received REST response: {json.dumps(response, indent=2)}")
            return response
            
        except zmq.error.Again:
            error_msg = f"REST request timed out after {self.timeout/1000.0} seconds"
            logger.error(error_msg)
            return {"error": error_msg}
        except Exception as e:
            error_msg = f"Error making REST request: {e}"
            logger.error(error_msg)
            return {"error": error_msg}
        finally:
            if socket:
                socket.close()

    async def make_request(self, command: str, params: Optional[Dict] = None) -> Dict:
        """
        Make a direct command request with proper message formatting.
        
        Args:
            command: Command name (e.g., 'ping', 'get_status', 'POST /v1/agent/register')
            params: Command parameters
            
        Returns:
            Response data or error dictionary
        """
        # Create a new socket for this request (important for REQ/REP state)
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        socket.setsockopt(zmq.SNDTIMEO, self.timeout)
        socket.setsockopt(zmq.LINGER, 0)
        
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
        
    async def send_heartbeat(self, agent_id: str = "heartbeat", agent_type: str = "external") -> Dict:
        """
        Send a heartbeat to FEAGI.
        
        This keeps the connection alive by letting FEAGI know the agent is still running.
        
        Args:
            agent_id: Agent ID to include in the heartbeat
            agent_type: Agent type to include in the heartbeat
            
        Returns:
            Response dictionary
        """
        return await self.make_request("heartbeat", {"agent_id": agent_id, "agent_type": agent_type})
        
    async def send_goodbye(self, agent_id: str = "goodbye", agent_type: str = "external") -> Dict:
        """
        Send a goodbye message to FEAGI.
        
        This lets FEAGI know the agent is disconnecting gracefully.
        
        Args:
            agent_id: Agent ID to include in the goodbye message
            agent_type: Agent type to include in the goodbye message
            
        Returns:
            Response dictionary
        """
        return await self.make_request("goodbye", {"agent_id": agent_id, "agent_type": agent_type}) 