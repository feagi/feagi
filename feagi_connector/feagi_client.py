#!/usr/bin/env python3
"""
FEAGI Client

A reliable client implementation for communicating with FEAGI 2.1.
This module focuses on the REQ/REP pattern for command-based API communication,
which is the most stable ZMQ interface available.
"""

import json
import logging
import os
import sys
import time
from typing import Dict, Any, Optional, Tuple, List

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_client")


class FeagiClient:
    """
    Client for communicating with FEAGI 2.1.
    
    This client uses the REQ/REP pattern on port 5555 for reliable command-based
    communication with FEAGI.
    """
    
    def __init__(self, host="127.0.0.1", port=5555, timeout=5000, auth_token=None):
        """
        Initialize the FEAGI client.
        
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
        self.context = zmq.Context.instance()
        self.next_id = 1
    
    def command(self, cmd_name: str, params: Optional[Dict] = None) -> Dict:
        """
        Send a command to FEAGI.
        
        Args:
            cmd_name: Command name (e.g., 'ping', 'get_status')
            params: Command parameters
            
        Returns:
            Response data or error dictionary
        """
        # Create a new socket for this request (important for REQ/REP state)
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        try:
            # Connect to FEAGI
            socket.connect(f"tcp://{self.host}:{self.port}")
            
            # Prepare request
            request_data = {
                'command': cmd_name,
                'id': self.next_id,
                'timestamp': int(time.time() * 1000),
                'params': params or {}
            }
            self.next_id += 1
            
            # Serialize and send request with proper format
            # [auth_token, content_type, request_data]
            request_bytes = json.dumps(request_data).encode('utf-8')
            content_type = "application/json"
            auth_token_bytes = self.auth_token.encode('utf-8') if self.auth_token else b""
            
            socket.send_multipart([
                auth_token_bytes,              # Auth token (empty if none)
                content_type.encode('utf-8'),  # Content type
                request_bytes                  # Request data
            ])
            
            # Wait for response
            try:
                response_parts = socket.recv_multipart()
                
                # FEAGI should respond with [content_type, response_data]
                if len(response_parts) < 2:
                    return {'error': 'Invalid response format'}
                
                # Parse the response
                resp_content_type = response_parts[0].decode('utf-8')
                resp_data = response_parts[1]
                
                if resp_content_type == "application/json":
                    try:
                        return json.loads(resp_data.decode('utf-8'))
                    except json.JSONDecodeError:
                        return {'error': 'Invalid JSON response'}
                else:
                    return {
                        'content_type': resp_content_type,
                        'raw_response': resp_data.decode('utf-8', errors='replace')
                    }
                
            except zmq.error.Again:
                return {'error': 'Timeout waiting for response'}
                
        except Exception as e:
            return {'error': str(e)}
            
        finally:
            socket.close()
    
    # Common commands
    
    def ping(self) -> Dict:
        """Check if FEAGI is running."""
        return self.command("ping")
    
    def get_status(self) -> Dict:
        """Get the current status of FEAGI."""
        return self.command("get_status")
    
    def get_performance(self) -> Dict:
        """Get performance metrics from FEAGI."""
        return self.command("get_performance")
    
    def is_running(self) -> bool:
        """Check if FEAGI is running and responsive."""
        response = self.ping()
        return response.get("pong") is True
    
    def get_simulation_state(self) -> Dict:
        """Get the current simulation state."""
        response = self.get_status()
        if "status" in response:
            return response["status"]
        return {"error": response.get("error", "Unknown error")}


if __name__ == "__main__":
    # Example usage
    client = FeagiClient()
    
    # Check if FEAGI is running
    if client.is_running():
        print("FEAGI is running!")
        
        # Get status
        status = client.get_simulation_state()
        print(f"Simulation running: {status.get('running', False)}")
        print(f"Current step: {status.get('step', 0)}")
        
        # Get performance
        perf = client.get_performance()
        if "performance" in perf:
            print(f"FPS: {perf['performance'].get('fps', 0)}")
            print(f"Active neurons: {perf['performance'].get('neurons_active', 0)}")
            print(f"Memory usage: {perf['performance'].get('memory_usage', 0)}")
    else:
        print("FEAGI is not running or not responding") 