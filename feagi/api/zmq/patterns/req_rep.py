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
ZeroMQ Request-Reply Implementation for FEAGI API

This module implements the REQ/REP pattern for ZeroMQ communication in FEAGI.
It provides:
- Request-Reply server for handling command requests
- Request-Reply client for sending commands
- Command routing and execution framework
- REST API support for unified interface
"""

import asyncio
import json

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import time
import uuid
from typing import Any, Dict, Optional

import zmq
import zmq.asyncio

from ...core.services.core_api_service import CoreAPIService
from ...utils.auth import validate_token
from ..serialization import deserialize_message, serialize_message


class RequestReplyServer:
    """
    ZeroMQ Request-Reply server implementation.

    This server handles command requests from clients using the REQ/REP pattern.
    """

    def __init__(
        self,
        core_api: CoreAPIService,
        host: str = "*",
        port: int = 5555,
        context: Optional[zmq.asyncio.Context] = None,
    ):
        """
        Initialize a new Request-Reply server.

        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to (default "*" to bind to all interfaces)
            port: Port number to bind to
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://{host}:{port}")

        # Command handlers
        self.command_handlers = {
            "ping": self._handle_ping,
            "get_status": self._handle_get_status,
            "get_performance": self._handle_get_performance,
        }

    async def start(self) -> None:
        """Start the request-reply server."""
        logger.info(f"Starting REP server on {self.host}:{self.port}")
        self.running = True

        # Store the current event loop for this method
        self._event_loop = asyncio.get_event_loop()

        # Start the request handler in the current loop
        self._handler_task = self._event_loop.create_task(
            self._request_handler()
        )

    async def stop(self) -> None:
        """Stop the request-reply server."""
        logger.info("Stopping REP server")
        self.running = False
        self.socket.close()

    async def _request_handler(self) -> None:
        """Main loop for handling client requests."""
        while self.running:
            try:
                # Wait for request
                request_data = await self.socket.recv_multipart()

                # Add compatibility layer for simple JSON messages (not multipart)
                if len(request_data) == 1:
                    # Simple message format - try to parse as JSON
                    try:
                        simple_request = json.loads(request_data[0].decode())
                        logger.debug(
                            f"Received simple format request: {simple_request}"
                        )

                        # Convert simple format to expected format
                        if (
                            "type" in simple_request
                            and simple_request["type"] == "status_request"
                        ):
                            # Handle status_request specially
                            result = await self._handle_get_status(
                                {"params": {}}
                            )
                            await self._send_response(result)
                            continue
                        elif "command" in simple_request:
                            # Already has command field, use as is
                            command = simple_request["command"]
                            if command in self.command_handlers:
                                result = await self.command_handlers[command](
                                    simple_request
                                )
                                await self._send_response(result)
                                continue
                            else:
                                await self._send_error(
                                    f"Unknown command: {command}"
                                )
                                continue
                    except json.JSONDecodeError:
                        # Not JSON - continue with normal processing
                        logger.debug(
                            "Received message is not JSON, continuing with standard processing"
                        )
                        pass

                # Standard processing - expecting [auth_token, content_type, request_data]
                if len(request_data) < 3:
                    logger.error(f"Invalid request format: {request_data}")
                    await self._send_error("Invalid request format")
                    continue

                auth_token = request_data[0].decode()
                content_type = request_data[1].decode()
                request = deserialize_message(request_data[2], content_type)

                logger.debug(f"Received request: {request}")

                # Validate authentication if token is provided
                if auth_token and not await validate_token(auth_token):
                    logger.warning(
                        f"Invalid authentication token: {auth_token}"
                    )
                    await self._send_error("Authentication failed")
                    continue

                # Process command
                command = request.get("command")
                if not command:
                    await self._send_error("Missing command field")
                    continue

                # Handle command
                if command in self.command_handlers:
                    try:
                        result = await self.command_handlers[command](request)
                        await self._send_response(result)
                    except Exception as e:
                        logger.error(f"Error handling command {command}: {e}")
                        await self._send_error(
                            f"Error handling command: {str(e)}"
                        )
                else:
                    logger.warning(f"Unknown command: {command}")
                    await self._send_error(f"Unknown command: {command}")

            except asyncio.CancelledError:
                logger.debug("Request handler cancelled")
                break
            except Exception as e:
                logger.error(f"Error processing request: {e}")
                try:
                    await self._send_error(f"Internal server error: {str(e)}")
                except Exception:
                    pass
                await asyncio.sleep(1)  # Avoid tight loop on errors

    async def _send_response(
        self, data: Any, content_type: str = "application/json"
    ) -> None:
        """Send a response to the client."""
        try:
            serialized_data = serialize_message(data, content_type)
            await self.socket.send_multipart(
                [content_type.encode(), serialized_data]
            )
        except Exception as e:
            logger.error(f"Error sending response: {e}")

    async def _send_error(
        self, error_message: str, content_type: str = "application/json"
    ) -> None:
        """Send an error response to the client."""
        error_data = {"error": error_message, "timestamp": time.time()}
        await self._send_response(error_data, content_type)

    async def _handle_ping(self, request: Dict) -> Dict:
        """Handle ping command."""
        return {"pong": True, "timestamp": time.time(), "version": "1.0"}

    async def _handle_get_status(self, request: Dict) -> Dict:
        """Handle get_status command."""
        status = await self.core_api.get_simulation_status()
        return {"status": status, "timestamp": time.time()}

    async def _handle_get_performance(self, request: Dict) -> Dict:
        """Handle get_performance command."""
        performance = await self.core_api.get_performance_stats()
        return {"performance": performance, "timestamp": time.time()}


class RequestReplyClient:
    """
    ZeroMQ Request-Reply client implementation.

    This client connects to a Request-Reply server and sends command requests.
    """

    def __init__(
        self,
        host: str,  # Remove hardcoded default - must be provided from configuration
        port: int = 5555,
        timeout: int = 30,
        context: Optional[zmq.asyncio.Context] = None,
    ):
        """
        Initialize a new Request-Reply client.

        Args:
            host: Server host address to connect to
            port: Server port to connect to
            context: Optional existing ZMQ context to use
            timeout: Request timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.context = context or zmq.asyncio.Context.instance()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{host}:{port}")

        # Unique client ID
        self.client_id = str(uuid.uuid4())

        # Auth token (if any)
        self.auth_token = None

    async def start(self) -> None:
        """Start the request-reply client."""
        logger.info(f"Starting REQ client to {self.host}:{self.port}")
        pass  # Nothing to start for client

    async def stop(self) -> None:
        """Stop the request-reply client."""
        logger.info("Stopping REQ client")
        self.socket.close()

    async def send_request(
        self,
        command: str,
        params: Optional[Dict] = None,
        content_type: str = "application/json",
    ) -> Dict:
        """
        Send a command request to the server.

        Args:
            command: Command to execute
            params: Command parameters
            content_type: Content type for serialization

        Returns:
            Server response data
        """
        request = {
            "command": command,
            "params": params or {},
            "client_id": self.client_id,
            "timestamp": time.time(),
        }

        serialized_data = serialize_message(request, content_type)

        await self.socket.send_multipart(
            [
                self.auth_token.encode() if self.auth_token else b"",
                content_type.encode(),
                serialized_data,
            ]
        )

        # Set timeout
        self.socket.setsockopt(zmq.RCVTIMEO, int(self.timeout * 1000))

        # Receive response
        try:
            response = await self.socket.recv_multipart()

            # Expecting [content_type, response_data]
            if len(response) < 2:
                logger.error(f"Invalid response format: {response}")
                return {"error": "Invalid response format"}

            resp_content_type = response[0].decode()
            resp_data = deserialize_message(response[1], resp_content_type)

            return resp_data

        except zmq.error.Again:
            logger.error(f"Request timed out after {self.timeout} seconds")
            return {"error": "Request timed out"}

    def set_auth_token(self, token: str) -> None:
        """
        Set the authentication token to use for requests.

        Args:
            token: Authentication token
        """
        self.auth_token = token


class RequestReplyManager:
    """
    Manager class for coordinating Request-Reply servers and clients.

    This class provides a unified interface for the FEAGI ZMQ server
    to manage REQ/REP patterns.
    """

    def __init__(
        self,
        core_api: CoreAPIService,
        host: str = "*",
        port: int = 5555,
        context: Optional[zmq.asyncio.Context] = None,
    ):
        """
        Initialize a new RequestReply Manager.

        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to
            port: Port number to bind to
            context: Optional existing ZMQ context to use
        """
        self.context = context or zmq.asyncio.Context.instance()
        self._port = port
        self.request_reply_server = RequestReplyServer(
            core_api=core_api, host=host, port=port, context=self.context
        )

    @property
    def port(self) -> int:
        """Get the port used by this manager's server."""
        return self._port

    async def start(self) -> None:
        """Start the RequestReply manager."""
        await self.request_reply_server.start()

    async def stop(self) -> None:
        """Stop the RequestReply manager."""
        await self.request_reply_server.stop()
