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

"""API Gateway for FEAGI.

This module implements the API Gateway for FEAGI, providing a central interface
to access core FEAGI functionality. It serves as a connection point between
different API interfaces (REST, ZMQ, etc.) and the underlying core components.
"""

import os
import threading
import time
from queue import Queue
from typing import Any, Dict, List, Optional, Union

from feagi.utils.logger import setup_logger

logger = setup_logger()

from feagi.api.core.services import CoreAPIService
from feagi.api.protocols.constants import ProtocolID
from feagi.api.protocols.translator import ByteStructureTranslator
from feagi.api.zmq.client import ZmqClient


class RateLimiter:
    """Rate limiter for API Gateway.

    Implements a token bucket algorithm for rate limiting.
    """

    def __init__(self, rate_limit: int, burst_limit: int):
        """Initialize rate limiter.

        Args:
            rate_limit: Maximum number of requests per second
            burst_limit: Maximum burst size
        """
        self.rate_limit = rate_limit
        self.burst_limit = burst_limit
        self.tokens = burst_limit
        self.last_refill_time = time.time()
        self.lock = threading.Lock()

    def allow_request(self) -> bool:
        """Check if a request is allowed under the rate limit.

        Returns:
            True if the request is allowed, False otherwise
        """
        with self.lock:
            # Refill tokens based on time elapsed
            now = time.time()
            elapsed = now - self.last_refill_time
            refill = int(elapsed * self.rate_limit)

            self.tokens = min(self.tokens + refill, self.burst_limit)
            self.last_refill_time = now

            # Check if request can be allowed
            if self.tokens > 0:
                self.tokens -= 1
                return True

            return False


class AgentConnection:
    """Represents a connection to an agent."""

    def __init__(
        self,
        agent_id: str,
        agent_type: str,
        protocol_versions: Dict[str, int],
        rate_limit: int = 100,
        burst_limit: int = 200,
    ):
        """Initialize agent connection.

        Args:
            agent_id: Unique agent identifier
            agent_type: Type of agent (e.g., "monitor", "robot")
            protocol_versions: Dictionary mapping protocol names to version numbers
            rate_limit: Maximum requests per second
            burst_limit: Maximum burst size
        """
        self.agent_id = agent_id
        self.agent_type = agent_type
        self.protocol_versions = protocol_versions
        self.rate_limiter = RateLimiter(
            rate_limit=rate_limit, burst_limit=burst_limit
        )
        self.last_heartbeat = time.time()
        self.connected = True
        self.capabilities = {}  # Sensory/motor capabilities


class APIGateway:
    """API Gateway for FEAGI.

    This class provides a central point of access to FEAGI's functionality for
    all API interfaces (REST, ZMQ, etc.). It manages connections to:

    1. Core API - Direct access to in-process core components
    2. ZMQ Client - Access to remote FEAGI processes via ZMQ

    Features:
    - Protocol translation and version negotiation
    - Environment-based configuration
    - Authentication and authorization
    - Request routing
    - Rate limiting and monitoring

    The gateway automatically determines whether to use local components or
    connect to remote ones based on environment variables.
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        """Implement the singleton pattern."""
        if cls._instance is None:
            cls._instance = super(APIGateway, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, core_api: Optional[CoreAPIService] = None):
        """Initialize the API Gateway if not already initialized.

        Args:
            core_api: Optional CoreAPIService instance. If not provided,
                     one will be created or obtained from the process manager.
        """
        if getattr(self, "_initialized", False):
            return

        self._initialized = True
        self._running = True
        self._core_api = core_api
        self._zmq_client = None
        self._rate_limiters = {}
        self._auth_handlers = {}

        # New components for protocol handling
        self._protocol_translator = ByteStructureTranslator()
        self._agent_connections: Dict[str, AgentConnection] = {}

        # Message queues for async processing
        self._incoming_queue: Queue = Queue()
        self._outgoing_queues: Dict[str, Queue] = {}

        # Only auto-detect if core_api wasn't explicitly provided
        if self._core_api is None:
            self._initialize_core_api()

        # Initialize ZMQ client if enabled
        self._initialize_zmq_client()

        # Start message processing threads
        self._start_message_processors()

    def _initialize_core_api(self):
        """Initialize the Core API service based on environment."""
        try:
            # First check if we're running as part of the main FEAGI process
            if os.environ.get("FEAGI_INITIALIZED") == "1":
                logger.info(
                    "Running in FEAGI main process, using Process Manager"
                )
                from feagi.process_manager import get_process_manager

                process_manager = get_process_manager()
                self._core_api = process_manager.get_core_api()
                logger.info("Core API obtained from Process Manager")
            else:
                # If not, check if we should create a local instance
                if os.environ.get("FEAGI_LOCAL_CORE", "0") == "1":
                    logger.info("Creating local Core API instance")
                    from feagi.core import create_core_api

                    self._core_api = create_core_api()
                    logger.info("Local Core API created")
        except ImportError:
            logger.warning(
                "Could not import Process Manager or create local Core API"
            )

        # Create mock if we couldn't get a real core API
        if self._core_api is None:
            logger.warning("Using mock Core API")
            from unittest.mock import MagicMock

            self._core_api = MagicMock()

    def _initialize_zmq_client(self):
        """Initialize the ZMQ client if enabled by environment variables."""
        if os.environ.get("FEAGI_ZMQ_ENABLED", "0") == "1":
            try:
                # Use configuration system instead of hardcoded fallbacks
                from feagi.config.toml_loader import (
                    get_host_config,
                    get_port_config,
                    load_feagi_config,
                )

                # Load configuration
                config = load_feagi_config()
                host_config = get_host_config(config)
                port_config = get_port_config(config)

                # Use validated configuration values
                zmq_host = host_config.zmq_host
                zmq_req_port = port_config.zmq_req_rep_port
                zmq_pub_port = port_config.zmq_pub_sub_port
                zmq_push_port = port_config.zmq_push_pull_port
                zmq_stream_port = port_config.zmq_sensory_port

                logger.info(f"Initializing ZMQ client to {zmq_host}")
                self._zmq_client = ZmqClient(
                    host=zmq_host,
                    req_port=zmq_req_port,
                    sub_port=zmq_pub_port,
                    push_port=zmq_push_port,
                    stream_port=zmq_stream_port,
                )
                logger.info("ZMQ client initialized")
            except Exception as e:
                logger.error(f"Failed to initialize ZMQ client: {e}")

    def _start_message_processors(self):
        """Start message processing threads."""
        # Incoming message processor
        threading.Thread(
            target=self._process_incoming_messages,
            daemon=True,
            name="APIGateway-IncomingProcessor",
        ).start()

    def _process_incoming_messages(self):
        """Process incoming messages from agents."""
        # Get configurable timeout values once at the start
        try:
            from feagi.config.toml_loader import (
                get_timeout_config,
                load_feagi_config,
            )

            config = load_feagi_config()
            timeout_config = get_timeout_config(config)
            queue_timeout = (
                timeout_config.polling_timeout / 1000.0
            )  # Convert ms to seconds
            error_delay = (
                timeout_config.polling_timeout / 10000.0
            )  # Small fraction of polling timeout
        except Exception:
            queue_timeout = (
                1.0  # @architecture:acceptable - emergency fallback
            )
            error_delay = 0.1  # @architecture:acceptable - emergency fallback

        while self._running:
            try:
                # Get message from queue with configurable timeout
                binary_data, agent_id, protocol_id, version = (
                    self._incoming_queue.get(timeout=queue_timeout)
                )

                # Decode message using protocol translator
                message = self._protocol_translator.decode_message(
                    binary_data, protocol_id, version
                )

                # Route to appropriate handler
                self._route_message_to_core(
                    agent_id, message, protocol_id, version
                )

                self._incoming_queue.task_done()
            except Exception as e:
                logger.error(f"Error processing incoming message: {str(e)}")
                time.sleep(error_delay)  # Prevent tight loop on error

    def _process_outgoing_messages(self, agent_id: str):
        """Process outgoing messages for an agent.

        Args:
            agent_id: Agent identifier
        """
        # Get configurable timeout values
        try:
            from feagi.config.toml_loader import (
                get_timeout_config,
                load_feagi_config,
            )

            config = load_feagi_config()
            timeout_config = get_timeout_config(config)
            queue_timeout = (
                timeout_config.polling_timeout / 1000.0
            )  # Convert ms to seconds
            error_delay = (
                timeout_config.polling_timeout / 10000.0
            )  # Small fraction of polling timeout
        except Exception:
            queue_timeout = (
                1.0  # @architecture:acceptable - emergency fallback
            )
            error_delay = 0.1  # @architecture:acceptable - emergency fallback

        while (
            agent_id in self._agent_connections
            and self._agent_connections[agent_id].connected
        ):
            try:
                queue = self._outgoing_queues.get(agent_id)
                if not queue:
                    time.sleep(error_delay)
                    continue

                binary_data, protocol_id = queue.get(timeout=queue_timeout)

                # Here we would send the binary data via ZMQ
                # This will be implemented when integrated with ZMQManager

                queue.task_done()

            except Exception as e:
                if str(e) != "Empty":  # Ignore empty queue exceptions
                    logger.error(
                        f"Error processing outgoing message for {agent_id}: {str(e)}"
                    )

                time.sleep(error_delay)

    def _route_message_to_core(
        self,
        agent_id: str,
        message: Any,
        protocol_id: ProtocolID,
        version: int,
    ) -> None:
        """Route a message to the appropriate Core API Service handler.

        Args:
            agent_id: Agent identifier
            message: Decoded message data
            protocol_id: Protocol identifier
            version: Protocol version
        """
        if not self._core_api:
            logger.error("Cannot route message: CoreAPIService not available")
            return

        try:
            # Route based on protocol and message type
            if protocol_id == ProtocolID.FCP:
                # For FCP, route based on command type
                if isinstance(message, dict):
                    command_type = message.get("command_type")
                    # payload = message.get("payload", {})  # Unused variable removed

                    if command_type == 1:  # REGISTER
                        # Handle registration via CoreAPIService
                        # (agent should be pre-registered via REST API)
                        pass

                    elif command_type == 2:  # DEREGISTER
                        # Handle deregistration
                        self.deregister_agent(agent_id)

                    elif command_type == 6:  # HEARTBEAT
                        # Update last heartbeat time
                        if agent_id in self._agent_connections:
                            self._agent_connections[
                                agent_id
                            ].last_heartbeat = time.time()

                    # Other command types...

            elif protocol_id == ProtocolID.FSMP:
                # For FSMP, route to sensory processing
                if isinstance(message, dict) and "sensory_data" in message:
                    sensory_data = message.get("sensory_data", {})
                    # Forward to core API
                    if hasattr(self._core_api, "process_sensory_data"):
                        self._core_api.process_sensory_data(
                            agent_id, sensory_data
                        )

            elif protocol_id == ProtocolID.FVP:
                # Visualization requests would be handled here
                pass

        except Exception as e:
            logger.error(f"Error routing message to core: {str(e)}")

    @property
    def core_api(self) -> CoreAPIService:
        """Get the Core API service."""
        return self._core_api

    @property
    def zmq_client(self) -> Optional[ZmqClient]:
        """Get the ZMQ client if available."""
        return self._zmq_client

    def register_agent(
        self,
        agent_id: str,
        agent_type: str,
        protocol_versions: Dict[str, Union[int, List[int]]],
        capabilities: Dict[str, Any] = None,
    ) -> Dict[str, int]:
        """Register an agent with the gateway.

        Args:
            agent_id: Unique agent identifier
            agent_type: Type of agent (e.g., "monitor", "robot")
            protocol_versions: Dictionary mapping protocol names to supported version numbers
                              (either a single int or a list of supported versions)
            capabilities: Dictionary of agent capabilities (e.g., sensors, actuators)

        Returns:
            Dictionary of compatible protocol versions

        Raises:
            ValueError: If agent is already registered or protocol versions are incompatible
        """
        if agent_id in self._agent_connections:
            raise ValueError(f"Agent {agent_id} is already registered")

        # Negotiate compatible protocol versions
        try:
            compatible_versions = self._protocol_translator.register_agent(
                agent_id, protocol_versions
            )
        except ValueError as e:
            logger.error(f"Failed to negotiate protocol versions: {str(e)}")
            raise

        # Create agent connection
        connection = AgentConnection(
            agent_id=agent_id,
            agent_type=agent_type,
            protocol_versions=compatible_versions,
        )

        # Store capabilities if provided
        if capabilities:
            connection.capabilities = capabilities

        self._agent_connections[agent_id] = connection
        self._outgoing_queues[agent_id] = Queue()

        # Start outgoing message processor for this agent
        threading.Thread(
            target=self._process_outgoing_messages,
            args=(agent_id,),
            daemon=True,
            name=f"APIGateway-OutgoingProcessor-{agent_id}",
        ).start()

        logger.info(
            f"Agent {agent_id} registered with compatible versions: {compatible_versions}"
        )

        # If core API is available, register the agent there too
        if self._core_api and hasattr(self._core_api, "register_agent"):
            self._core_api.register_agent(
                agent_id=agent_id,
                agent_type=agent_type,
                protocol_versions=compatible_versions,
                capabilities=capabilities or {},
            )

        return compatible_versions

    def deregister_agent(self, agent_id: str) -> bool:
        """Deregister an agent.

        Args:
            agent_id: Agent identifier

        Returns:
            True if agent was deregistered, False if agent was not registered
        """
        if agent_id not in self._agent_connections:
            return False

        # Mark as disconnected
        self._agent_connections[agent_id].connected = False

        # Remove from protocol translator
        self._protocol_translator.deregister_agent(agent_id)

        # Remove from agent connections
        del self._agent_connections[agent_id]

        # Cleanup outgoing queue
        if agent_id in self._outgoing_queues:
            del self._outgoing_queues[agent_id]

        # If core API is available, deregister the agent there too
        if self._core_api and hasattr(self._core_api, "deregister_agent"):
            self._core_api.deregister_agent(agent_id)

        logger.info(f"Agent {agent_id} deregistered")
        return True

    def receive_message(self, agent_id: str, binary_data: bytes) -> bool:
        """Receive a binary message from an agent.

        Args:
            agent_id: Agent identifier
            binary_data: Raw binary message data

        Returns:
            True if message was accepted, False if rejected (e.g., rate limited)
        """
        # Check if agent exists
        if agent_id not in self._agent_connections:
            logger.warning(f"Message received from unknown agent {agent_id}")
            return False

        connection = self._agent_connections[agent_id]

        # Apply rate limiting
        if not connection.rate_limiter.allow_request():
            logger.warning(f"Rate limit exceeded for agent {agent_id}")
            return False

        try:
            # Decode message using protocol translator
            decoded_data, protocol_id, version = (
                self._protocol_translator.decode(binary_data)
            )

            # Queue message for processing
            self._incoming_queue.put(
                (agent_id, decoded_data, protocol_id, version)
            )
            return True

        except Exception as e:
            logger.error(
                f"Error decoding message from agent {agent_id}: {str(e)}"
            )
            return False

    def send_message(
        self, agent_id: str, message: Any, protocol_name: str
    ) -> bool:
        """Send a message to an agent.

        Args:
            agent_id: Agent identifier
            message: Message data to send
            protocol_name: Protocol to use

        Returns:
            True if message was queued for sending, False otherwise
        """
        # Check if agent exists
        if agent_id not in self._agent_connections:
            logger.warning(f"Cannot send message to unknown agent {agent_id}")
            return False

        try:
            # Encode message
            binary_data = self._protocol_translator.encode(
                agent_id, message, protocol_name
            )
            protocol_id = ProtocolID[protocol_name]

            # Queue for sending
            if agent_id in self._outgoing_queues:
                self._outgoing_queues[agent_id].put((binary_data, protocol_id))
                return True

            return False

        except Exception as e:
            logger.error(
                f"Error encoding message for agent {agent_id}: {str(e)}"
            )
            return False

    def get_agent_connection(self, agent_id: str) -> Optional[AgentConnection]:
        """Get information about an agent connection.

        Args:
            agent_id: Agent identifier

        Returns:
            AgentConnection object if agent is registered, None otherwise
        """
        return self._agent_connections.get(agent_id)

    def get_agent_status(self, agent_id: str) -> Dict[str, Any]:
        """Get status information about an agent.

        Args:
            agent_id: Agent identifier

        Returns:
            Dictionary with agent status information
        """
        connection = self._agent_connections.get(agent_id)

        if not connection:
            return {"error": f"Agent {agent_id} not found"}

        return {
            "agent_id": agent_id,
            "agent_type": connection.agent_type,
            "connected": connection.connected,
            "last_heartbeat": connection.last_heartbeat,
            "protocols": connection.protocol_versions,
            "capabilities": connection.capabilities,
        }

    def get_all_agents(self) -> Dict[str, Dict[str, Any]]:
        """Get information about all registered agents.

        Returns:
            Dictionary mapping agent IDs to agent information
        """
        return {
            agent_id: self.get_agent_status(agent_id)
            for agent_id in self._agent_connections
        }

    # Authentication and authorization methods

    def authenticate(self, credentials: Dict[str, Any]) -> bool:
        """Authenticate a client.

        Args:
            credentials: Dictionary containing authentication credentials.

        Returns:
            True if authentication is successful, False otherwise.
        """
        # Placeholder for authentication implementation
        return True

    def authorize(
        self, resource: str, action: str, credentials: Dict[str, Any]
    ) -> bool:
        """Authorize a client to perform an action on a resource.

        Args:
            resource: The resource being accessed.
            action: The action being performed.
            credentials: Dictionary containing authentication credentials.

        Returns:
            True if authorization is successful, False otherwise.
        """
        # Placeholder for authorization implementation
        return True

    # Routing methods

    def route_request(
        self, protocol: str, endpoint: str, method: str, data: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Route a request to the appropriate handler.

        Args:
            protocol: The protocol being used (REST, ZMQ).
            endpoint: The endpoint being accessed.
            method: The HTTP method or ZMQ pattern.
            data: Dictionary containing request data.

        Returns:
            Dictionary containing response data.
        """
        # Implementation will route to appropriate handler based on protocol and endpoint
        pass

    # Rate limiting methods

    def check_rate_limit(self, client_id: str, endpoint: str) -> bool:
        """Check if a client has exceeded rate limits for an endpoint.

        Args:
            client_id: ID of the client.
            endpoint: The endpoint being accessed.

        Returns:
            True if the client has not exceeded rate limits, False otherwise.
        """
        # Use the agent's rate limiter if available
        if client_id in self._agent_connections:
            return self._agent_connections[
                client_id
            ].rate_limiter.allow_request()

        # Otherwise use a default rate limiter
        if client_id not in self._rate_limiters:
            self._rate_limiters[client_id] = RateLimiter(
                rate_limit=10, burst_limit=20
            )

        return self._rate_limiters[client_id].allow_request()

    # Monitoring methods

    def record_request(
        self, protocol: str, endpoint: str, status: int, duration: float
    ):
        """Record a request for monitoring purposes.

        Args:
            protocol: The protocol being used (REST, ZMQ).
            endpoint: The endpoint being accessed.
            status: The response status.
            duration: The request duration in seconds.
        """
        # Implementation will record request metrics
        pass

    def get_metrics(self) -> Dict[str, Any]:
        """Get API metrics.

        Returns:
            Dictionary containing API metrics.
        """
        # Placeholder for metrics implementation
        return {
            "agents": {
                "count": len(self._agent_connections),
                "types": self._get_agent_type_counts(),
            },
            "messages": {
                "incoming_queue_size": self._incoming_queue.qsize(),
                "outgoing_queues": {
                    agent_id: queue.qsize()
                    for agent_id, queue in self._outgoing_queues.items()
                },
            },
        }

    def _get_agent_type_counts(self) -> Dict[str, int]:
        """Get counts of registered agent types."""
        type_counts = {}

        for agent in self._agent_connections.values():
            agent_type = agent.agent_type
            type_counts[agent_type] = type_counts.get(agent_type, 0) + 1

        return type_counts


# Factory function for creating/getting gateway instances
def get_api_gateway(core_api: Optional[CoreAPIService] = None) -> APIGateway:
    """Get or create an API Gateway instance.

    Args:
        core_api: Optional CoreAPIService instance to use in the gateway.
                If not provided, the gateway will auto-detect appropriate
                Core API access based on environment.

    Returns:
        APIGateway instance (singleton).
    """
    return APIGateway(core_api)
