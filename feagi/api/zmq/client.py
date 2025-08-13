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

"""ZeroMQ Client for FEAGI API.

This module implements a ZeroMQ client for connecting to a remote FEAGI ZMQ server.
"""

import asyncio

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import time
from typing import Any, Callable, Dict, List, Optional

import zmq
import zmq.asyncio


class ZmqClient:
    """
    ZeroMQ client for connecting to a remote FEAGI ZMQ server.

    This client connects to all ZMQ patterns provided by the FEAGI ZMQ server:
    - Request-Reply: For traditional CRUD operations
    - Publish-Subscribe: For receiving events and updates
    - Push-Pull: For high-throughput data processing
    - Sensorimotor Stream: For efficient binary exchange of sensorimotor data
    """

    def __init__(
        self,
        host: str,
        req_port: int = 5555,
        pub_port: int = 5556,
        push_port: int = 5557,
        stream_port: int = 5558,
        timeout: int = 30000,
        context: Optional[zmq.Context] = None,
        topics: Optional[List[str]] = None,
        stream_topics: Optional[List[str]] = None,
    ):
        """
        Initialize a new ZMQ client.

        Args:
            host: Host address to connect to
            req_port: Port for Request-Reply pattern
            pub_port: Port for Publish-Subscribe pattern
            push_port: Port for Push (to server's Pull)
            stream_port: Port for Sensorimotor stream
            timeout: Timeout for requests in milliseconds
            context: Optional existing ZMQ context to use
            topics: List of topics to subscribe to
            stream_topics: List of stream topics to subscribe to
        """
        self.host = host
        self.req_port = req_port
        self.pub_port = pub_port
        self.push_port = push_port
        self.stream_port = stream_port
        self.timeout = timeout

        # Initialize ZMQ context
        self.context = context or zmq.Context.instance()

        # Initialize sockets
        self._req_socket = None
        self._sub_socket = None
        self._push_socket = None
        self._stream_socket = None

        # Tracking event callbacks
        self._event_callbacks = {}
        self._running = False
        self._event_task = None

        # Topics
        self.topics = topics or []
        self.stream_topics = stream_topics or []

    async def connect(self):
        """Connect to the ZMQ server."""
        logger.info(f"Connecting to ZMQ server at {self.host}")

        # Create Request socket
        self._req_socket = self.context.socket(zmq.REQ)
        self._req_socket.connect(f"tcp://{self.host}:{self.req_port}")

        # Create Subscribe socket
        self._sub_socket = self.context.socket(zmq.SUB)
        self._sub_socket.connect(f"tcp://{self.host}:{self.pub_port}")

        # Create Push socket
        self._push_socket = self.context.socket(zmq.PUSH)
        self._push_socket.connect(f"tcp://{self.host}:{self.push_port}")

        # Create Stream socket
        self._stream_socket = self.context.socket(zmq.STREAM)
        self._stream_socket.connect(f"tcp://{self.host}:{self.stream_port}")

        # Start event listener
        self._running = True
        self._event_task = asyncio.create_task(self._listen_for_events())

        logger.info("Connected to ZMQ server")
        return True

    async def disconnect(self):
        """Disconnect from the ZMQ server."""
        logger.info("Disconnecting from ZMQ server")

        # Stop event listener
        self._running = False
        if self._event_task:
            self._event_task.cancel()
            try:
                await self._event_task
            except asyncio.CancelledError:
                pass

        # Close sockets
        for socket in [
            self._req_socket,
            self._sub_socket,
            self._push_socket,
            self._stream_socket,
        ]:
            if socket:
                socket.close()

        self._req_socket = None
        self._sub_socket = None
        self._push_socket = None
        self._stream_socket = None

        logger.info("Disconnected from ZMQ server")

    async def request(
        self, action: str, data: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Send a request to the server and wait for a response.

        Args:
            action: The action to perform
            data: Additional data for the request

        Returns:
            The response from the server

        Raises:
            TimeoutError: If the server doesn't respond within the timeout
        """
        if self._req_socket is None:
            await self.connect()

        request_data = {
            "action": action,
            "data": data or {},
            "timestamp": time.time(),
        }

        await self._req_socket.send_json(request_data)

        # Wait for response with timeout
        poller = zmq.Poller()
        poller.register(self._req_socket, zmq.POLLIN)

        if poller.poll(int(self.timeout / 1000)):
            response = await self._req_socket.recv_json()
            return response
        else:
            raise TimeoutError(
                f"Request timed out after {self.timeout / 1000} seconds"
            )

    async def subscribe(
        self, topic: str, callback: Callable[[Dict[str, Any]], None]
    ):
        """
        Subscribe to a topic and register a callback for events.

        Args:
            topic: Topic to subscribe to
            callback: Function to call when an event is received
        """
        if self._sub_socket is None:
            await self.connect()

        self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, topic)

        if topic not in self._event_callbacks:
            self._event_callbacks[topic] = []

        self._event_callbacks[topic].append(callback)

    async def unsubscribe(
        self, topic: str, callback: Optional[Callable] = None
    ):
        """
        Unsubscribe from a topic.

        Args:
            topic: Topic to unsubscribe from
            callback: Specific callback to remove, or None to remove all
        """
        if self._sub_socket is None:
            return

        if callback is None:
            # Remove all callbacks for this topic
            self._event_callbacks.pop(topic, None)
            self._sub_socket.setsockopt_string(zmq.UNSUBSCRIBE, topic)
        else:
            # Remove specific callback
            if topic in self._event_callbacks:
                try:
                    self._event_callbacks[topic].remove(callback)
                except ValueError:
                    pass

                # If no more callbacks for this topic, unsubscribe
                if not self._event_callbacks[topic]:
                    self._event_callbacks.pop(topic)
                    self._sub_socket.setsockopt_string(zmq.UNSUBSCRIBE, topic)

    async def push(self, data: Dict[str, Any]):
        """
        Push data to the server.

        Args:
            data: Data to push
        """
        if self._push_socket is None:
            await self.connect()

        await self._push_socket.send_json(data)

    async def _listen_for_events(self):
        """Listen for events from the server and dispatch to callbacks."""
        try:
            while self._running:
                if self._sub_socket is None:
                    await asyncio.sleep(0.1)
                    continue

                try:
                    [topic, data] = await self._sub_socket.recv_multipart()
                    topic_str = topic.decode("utf-8")

                    # Process the event in all registered callbacks
                    if topic_str in self._event_callbacks:
                        event_data = data.decode("utf-8")
                        for callback in self._event_callbacks[topic_str]:
                            try:
                                callback(event_data)
                            except Exception as e:
                                logger.error(f"Error in event callback: {e}")

                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error listening for events: {e}")
                    await asyncio.sleep(1.0)

        except asyncio.CancelledError:
            pass


def create_zmq_client(
    host: str,
    req_port: int = 5555,
    pub_port: int = 5556,
    push_port: int = 5557,
    stream_port: int = 5558,
    topics: Optional[List[str]] = None,
    stream_topics: Optional[List[str]] = None,
) -> ZmqClient:
    """
    Create and start a ZeroMQ client.

    Args:
        host: Host address of the server to connect to.
        req_port: Port for Request-Reply socket.
        pub_port: Port for Publish-Subscribe socket.
        push_port: Port for Push-Pull socket.
        stream_port: Port for Stream socket.
        topics: List of topics to subscribe to.
        stream_topics: List of stream topics to subscribe to.

    Returns:
        Running ZmqClient instance.
    """
    client = ZmqClient(
        host=host,
        req_port=req_port,
        pub_port=pub_port,
        push_port=push_port,
        stream_port=stream_port,
        topics=topics,
        stream_topics=stream_topics,
    )
    client.start()
    return client
