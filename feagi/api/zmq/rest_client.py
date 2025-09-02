"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
ZMQ REST API Client

This module provides a client implementation for the ZMQ REST API protocol,
allowing applications to use the same API interface over ZMQ as they would
over HTTP.
"""

import json
import time
import uuid
from typing import Any, Dict, List, Optional

import zmq


class ZMQRestClient:
    """Client for accessing FEAGI REST API over ZMQ.

    This client provides methods that mirror the HTTP REST API but use ZMQ as
    the transport protocol instead of HTTP.
    """

    def __init__(self, host: str, port: int = 5555, timeout: int = 30):
        """Initialize the ZMQ REST client.

        Args:
            host: FEAGI ZMQ server host (required - no hardcoded defaults)
            port: FEAGI ZMQ control port
            timeout: Request timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.context = zmq.Context.instance()
        self.socket = None
        self.identity = str(uuid.uuid4()).encode("utf-8")

    def connect(self):
        """Connect to the FEAGI ZMQ server.

        Raises:
            ConnectionError: If connection fails
        """
        if self.socket:
            return

        try:
            # Create socket
            self.socket = self.context.socket(zmq.DEALER)
            self.socket.setsockopt(zmq.IDENTITY, self.identity)

            # Set timeout
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout * 1000)

            # Connect
            self.socket.connect(f"tcp://{self.host}:{self.port}")
        except zmq.ZMQError as e:
            raise ConnectionError(
                f"Failed to connect to FEAGI ZMQ server: {e}"
            ) from e

    def disconnect(self):
        """Disconnect from the FEAGI ZMQ server."""
        if self.socket:
            self.socket.close()
            self.socket = None

    def request(
        self,
        method: str,
        route: str,
        params: Optional[Dict[str, Any]] = None,
        query: Optional[Dict[str, Any]] = None,
        body: Optional[Dict[str, Any]] = None,
        headers: Optional[Dict[str, str]] = None,
    ) -> Dict[str, Any]:
        """Send a request to the FEAGI ZMQ server.

        Args:
            method: HTTP method (GET, POST, PUT, DELETE)
            route: API route (e.g., '/v1/genome/blueprint')
            params: Path parameters
            query: Query parameters
            body: Request body
            headers: Request headers

        Returns:
            Response as a dictionary

        Raises:
            ConnectionError: If not connected or connection fails
            TimeoutError: If request times out
            ValueError: If response format is invalid
        """
        # Connect if not already connected
        if not self.socket:
            self.connect()

        # Create request
        request = {
            "method": method,
            "route": route,
            "timestamp": int(time.time() * 1000),
        }

        # Add optional components
        if params:
            request["params"] = params
        if query:
            request["query"] = query
        if body:
            request["body"] = body
        if headers:
            request["headers"] = headers

        # Send request
        try:
            # Convert request to bytes
            request_bytes = json.dumps(request).encode("utf-8")

            # Send as multipart message for DEALER/ROUTER pattern
            # [empty_frame, payload]
            self.socket.send_multipart([b"", request_bytes])
        except zmq.ZMQError as e:
            raise ConnectionError(f"Failed to send request: {e}") from e

        # Wait for response
        try:
            # Receive multipart response
            response_parts = self.socket.recv_multipart()

            # Parse response (should be [empty_frame, payload])
            if len(response_parts) < 2:
                raise ValueError("Invalid response format: missing parts")

            # Get JSON payload
            response = json.loads(response_parts[1].decode("utf-8"))
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                raise TimeoutError("Request timed out") from e
            else:
                raise ConnectionError(
                    f"Failed to receive response: {e}"
                ) from e

        # Check response format
        if not isinstance(response, dict):
            raise ValueError(f"Invalid response format: {response}")

        # Check for error
        if response.get("status", 200) >= 400:
            error = response.get("body", {})
            message = error.get("message", "Unknown error")
            code = error.get("code", "ERROR")
            raise RuntimeError(f"{code}: {message}")

        # Return response body
        return response.get("body", {})

    # Convenience methods for common HTTP methods

    def get(
        self,
        route: str,
        params: Optional[Dict[str, Any]] = None,
        query: Optional[Dict[str, Any]] = None,
        headers: Optional[Dict[str, str]] = None,
    ) -> Dict[str, Any]:
        """Send a GET request.

        Args:
            route: API route
            params: Path parameters
            query: Query parameters
            headers: Request headers

        Returns:
            Response body
        """
        return self.request("GET", route, params, query, None, headers)

    def post(
        self,
        route: str,
        body: Dict[str, Any],
        params: Optional[Dict[str, Any]] = None,
        query: Optional[Dict[str, Any]] = None,
        headers: Optional[Dict[str, str]] = None,
    ) -> Dict[str, Any]:
        """Send a POST request.

        Args:
            route: API route
            body: Request body
            params: Path parameters
            query: Query parameters
            headers: Request headers

        Returns:
            Response body
        """
        return self.request("POST", route, params, query, body, headers)

    def put(
        self,
        route: str,
        body: Dict[str, Any],
        params: Optional[Dict[str, Any]] = None,
        query: Optional[Dict[str, Any]] = None,
        headers: Optional[Dict[str, str]] = None,
    ) -> Dict[str, Any]:
        """Send a PUT request.

        Args:
            route: API route
            body: Request body
            params: Path parameters
            query: Query parameters
            headers: Request headers

        Returns:
            Response body
        """
        return self.request("PUT", route, params, query, body, headers)

    def delete(
        self,
        route: str,
        params: Optional[Dict[str, Any]] = None,
        query: Optional[Dict[str, Any]] = None,
        headers: Optional[Dict[str, str]] = None,
    ) -> Dict[str, Any]:
        """Send a DELETE request.

        Args:
            route: API route
            params: Path parameters
            query: Query parameters
            headers: Request headers

        Returns:
            Response body
        """
        return self.request("DELETE", route, params, query, None, headers)

    # System endpoints

    def get_health(self) -> Dict[str, Any]:
        """Get FEAGI system health status."""
        return self.get("/v1/system/health_check")

    def get_configuration(self) -> Dict[str, Any]:
        """Get FEAGI configuration."""
        return self.get("/v1/system/configuration")

    def update_configuration(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """Update FEAGI configuration."""
        return self.put("/v1/system/configuration", config)

    def get_versions(self) -> Dict[str, Any]:
        """Get FEAGI component versions."""
        return self.get("/v1/system/versions")

    def get_cortical_area_types(self) -> Dict[str, Any]:
        """Get available cortical area types."""
        return self.get("/v1/system/cortical_area_types")

    # Genome endpoints

    def get_genome(self) -> Dict[str, Any]:
        """Get the current genome."""
        return self.get("/v1/genome")

    def get_genome_blueprint(self) -> Dict[str, Any]:
        """Get the genome blueprint (cortical areas)."""
        return self.get("/v1/genome/blueprint")

    # Connectome endpoints

    def get_cortical_areas(self) -> List[Dict[str, Any]]:
        """Get all cortical areas."""
        return self.get("/v1/connectome/cortical_areas")

    def get_cortical_area(self, cortical_id: str) -> Dict[str, Any]:
        """Get a specific cortical area using the correct FEAGI endpoint.

        Args:
            cortical_id: Cortical area ID

        Returns:
            Cortical area details
        """
        return self.post(
            "/v1/cortical_area/cortical_area_properties",
            body={"cortical_id": cortical_id},
        )

    # Status endpoint

    def get_status(self) -> Dict[str, Any]:
        """Get FEAGI system status."""
        return self.get("/v1/status")
