"""
ZeroMQ Interface for FEAGI API

This package provides ZeroMQ-based interfaces for high-performance
communication with FEAGI, including:

- Request-Reply pattern for CRUD operations
- Publish-Subscribe pattern for events and updates
- Push-Pull pattern for high-throughput data processing
- Specialized streams for sensory data, motor control, and visualization
"""

import os

from feagi.utils.logger import setup_logger

logger = setup_logger()
import asyncio
import threading
from typing import Any, Callable, List, Optional

from .client import ZmqClient
from .patterns import (
    PublisherServer,
    PubSubManager,
    PullClient,
    PushPullManager,
    PushServer,
    RequestReplyClient,
    RequestReplyManager,
    RequestReplyServer,
    SubscriberClient,
)
from .server import ZmqServer
from .streams import MotorStream, RestStream, SensoryStream, VisualizationStream

# Export top-level classes
__all__ = [
    # Server
    "ZmqServer",
    "create_zmq_server",
    # Client
    "ZmqClient",
    # Patterns
    "RequestReplyServer",
    "RequestReplyClient",
    "RequestReplyManager",
    "PublisherServer",
    "SubscriberClient",
    "PubSubManager",
    "PushServer",
    "PullClient",
    "PushPullManager",
    # Streams
    "SensoryStream",
    "MotorStream",
    "VisualizationStream",
    "RestStream",
]


def create_zmq_server(
    host: str = None,
    sensory_port: int = None,
    motor_port: int = None,
    control_port: int = None,
    visualization_port: int = None,
    use_auth=False,
    use_encryption=False,
    config=None,
):
    """
    Create and initialize a ZMQ server instance.

    This is the main factory function for creating ZMQ servers in FEAGI.

    Args:
        host: Host address to bind to
        sensory_port: Port for the sensory socket
        motor_port: Port for the motor socket
        control_port: Port for the control socket
        visualization_port: Port for the visualization socket
        use_auth: Whether to use authentication (deprecated)
        use_encryption: Whether to use encryption (deprecated)
        config: Additional configuration (deprecated)

    Returns:
        Initialized ZmqServer instance or None if initialization fails
    """
    # Import here to avoid circular import issues with server module
    from .connection_manager import ZMQConnectionManager

    server = None

    try:
        # Get connection manager instance
        conn_manager = ZMQConnectionManager.instance(host=host)

        # Create server with specified parameters
        kwargs = {}
        if host is not None:
            kwargs["host"] = host
        if sensory_port is not None:
            kwargs["sensory_port"] = sensory_port
        if motor_port is not None:
            kwargs["motor_port"] = motor_port
        if control_port is not None:
            kwargs["control_port"] = control_port
        if visualization_port is not None:
            kwargs["vis_port"] = visualization_port

        server = conn_manager.create_server(server_type="default", **kwargs)

    except Exception as e:
        logger.error(f"Failed to create ZMQ server: {e}")
        return None

    return server


def create_zmq_client(
    host: str = None,
    pub_port: int = None,
    sub_port: int = None,
    topics: List[str] = None,
):
    """
    Create a ZMQ client for connecting to a FEAGI ZMQ server.

    This function creates a client that can publish and subscribe to
    the specified ZMQ server.

    Args:
        host: Host address of the ZMQ server to connect to
        pub_port: Publisher port to connect to
        sub_port: Subscriber port to connect to
        topics: List of topics to subscribe to

    Returns:
        A ZMQ client instance or None if creation fails
    """
    try:
        # Check if the server is already running in this process
        try:
            from feagi.main import get_zmq_client

            client = get_zmq_client()
            if client is not None and client.running:
                logger.info("Using in-process ZMQ server as client")
                return client
        except (ImportError, AttributeError):
            pass

        # Create the client
        # Use configuration system instead of hardcoded fallback
        if host is None:
            from feagi.config.toml_loader import get_host_config, load_feagi_config

            config = load_feagi_config()
            host_config = get_host_config(config)
            host = host_config.zmq_host

        client = ZmqClient(
            host=host,
            req_port=5555,  # Default req_port
            pub_port=pub_port or int(os.environ.get("FEAGI_ZMQ_PUB_PORT", "5556")),
            push_port=sub_port or int(os.environ.get("FEAGI_ZMQ_SUB_PORT", "5557")),
            stream_port=5558,
            topics=topics,
        )

        # Create a wrapper class to mimic the old ZMQClient API
        class ZmqClientWrapper:
            def __init__(self, client):
                self.client = client
                self.running = False
                self.subscriptions = {}

            def start(self):
                """Start the client"""

                # Create and run a background thread to run the client
                def run_client():
                    self.client.start()

                self.thread = threading.Thread(target=run_client, daemon=True)
                self.thread.start()
                self.running = True
                return True

            def shutdown(self):
                """Shutdown the client"""
                if self.running:
                    # Create a new event loop for shutdown
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    loop.run_until_complete(self.client.stop())
                    loop.close()
                    self.running = False

            def publish(self, topic, message):
                """Publish a message to a topic"""
                # Forward to ZmqClient
                response = self.client.send_request(
                    "publish", {"topic": topic, "message": message}
                )
                return "error" not in response

            def subscribe(self, topic, callback):
                """Subscribe to a topic with a callback"""
                # Store subscription for later use
                self.subscriptions[topic] = callback

                # Create function that accepts data only
                def topic_callback(data):
                    # Call the original callback with topic and data
                    callback(topic, data)

                # Register callback
                self.client.register_topic_callback(topic, topic_callback)
                return True

        # Create and start the wrapper
        wrapper = ZmqClientWrapper(client)
        wrapper.start()
        return wrapper

    except Exception as e:
        logger.error(f"Failed to create ZMQ client: {e}")
        return None
