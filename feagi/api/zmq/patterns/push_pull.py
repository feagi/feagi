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
ZeroMQ Push-Pull Implementation for FEAGI API

This module implements the PUSH/PULL pattern for ZeroMQ communication in FEAGI.
It provides:
- Push server for distributing work items to multiple workers
- Pull client for receiving and processing work items
- Load balancing across multiple workers
- Backpressure handling for overloaded workers
"""

import asyncio

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import time
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import zmq
import zmq.asyncio

from ...core.services.core_api_service import CoreAPIService
from ...utils.rate_limit import RateLimiter
from ..serialization import deserialize_message, serialize_message


class PushServer:
    """
    ZeroMQ Push server implementation.

    This server distributes work items to one or more Pull clients using the PUSH/PULL pattern.
    It's designed for load balancing work across multiple workers.
    """

    def __init__(
        self,
        core_api: CoreAPIService,
        host: str = "*",
        port: int = 5557,
        hwm: int = 1000,
        context: Optional[zmq.asyncio.Context] = None,
    ):
        """
        Initialize a new Push server.

        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to (default "*" to bind to all interfaces)
            port: Port number to bind to
            hwm: High water mark (max queued messages)
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.setsockopt(zmq.SNDHWM, hwm)
        self.socket.bind(f"tcp://{host}:{port}")
        self.rate_limiter = RateLimiter()

        # Queue for pending work items
        self.work_queue = asyncio.Queue()

        # Statistics
        self.stats = {"sent_items": 0, "queued_items": 0, "last_push_time": 0}

    async def start(self) -> None:
        """Start the push server."""
        logger.info(f"Starting PUSH server on {self.host}:{self.port}")
        self.running = True
        # Store the task reference for proper shutdown
        self._process_task = asyncio.create_task(self._process_queue())

    async def stop(self) -> None:
        """Stop the push server with RTOS-friendly cleanup."""
        logger.info("Stopping PUSH server")
        self.running = False

        # Cancel and wait for the processing task
        if hasattr(self, "_process_task"):
            self._process_task.cancel()
            try:
                await self._process_task
            except asyncio.CancelledError:
                logger.debug("Push server process task cancelled successfully")
            except Exception as e:
                logger.warning(f"Error cancelling push server task: {e}")

        # Close socket
        if self.socket:
            try:
                self.socket.close()
            except Exception as e:
                logger.warning(f"Error closing push server socket: {e}")

    async def push_item(
        self,
        item: Any,
        work_type: str = "default",
        priority: int = 0,
        content_type: str = "application/json",
    ) -> None:
        """
        Queue a work item for processing.

        Args:
            item: The work item data to push
            work_type: Type of work for categorization
            priority: Priority (higher values are processed first)
            content_type: Content type for serialization
        """
        await self.work_queue.put((priority, work_type, item, content_type))
        self.stats["queued_items"] = self.work_queue.qsize()
        logger.debug(f"Queued work item of type {work_type}, priority {priority}")

    async def _process_queue(self) -> None:
        """Process queued work items and push them to workers with RTOS-friendly error handling."""
        while self.running:
            try:
                # Get the next work item (blocks until one is available)
                # Use a timeout to allow for clean cancellation
                try:
                    priority, work_type, item, content_type = await asyncio.wait_for(
                        self.work_queue.get(), timeout=0.5
                    )
                except asyncio.TimeoutError:
                    # Check if we should still be running
                    if not self.running:
                        break
                    continue

                # Prepare the message
                serialized_data = serialize_message(item, content_type)
                message = [work_type.encode(), content_type.encode(), serialized_data]

                # Send the message
                await self.socket.send_multipart(message)

                # Update statistics
                self.stats["sent_items"] += 1
                self.stats["queued_items"] = self.work_queue.qsize()
                self.stats["last_push_time"] = time.time()

                logger.debug(f"Pushed work item of type {work_type}")

                # Mark task as done
                self.work_queue.task_done()

            except asyncio.CancelledError:
                logger.debug("Process queue task cancelled")
                break
            except Exception as e:
                logger.error(f"Error processing work queue: {e}")
                # RTOS-friendly: Simple check instead of sleep
                if not self.running:
                    break
                # Use asyncio.sleep for proper async error recovery
                try:
                    await asyncio.sleep(
                        0.1
                    )  # @architecture:acceptable - error recovery
                except asyncio.CancelledError:
                    # Handle cancellation during error recovery
                    logger.debug("Process queue cancelled during error recovery")
                    break

    async def push_batch(
        self, items: List[Tuple[Any, str, int]], content_type: str = "application/json"
    ) -> None:
        """
        Queue multiple work items for processing.

        Args:
            items: List of (item, work_type, priority) tuples
            content_type: Content type for serialization
        """
        for item, work_type, priority in items:
            await self.push_item(item, work_type, priority, content_type)


class PullClient:
    """
    ZeroMQ Pull client implementation.

    This client connects to a Push server and receives work items for processing.
    """

    def __init__(
        self,
        host: str,  # Remove hardcoded default - must be provided from configuration
        port: int = 5557,
        hwm: int = 100,
        context: Optional[zmq.asyncio.Context] = None,
    ):
        """
        Initialize a new Pull client.

        Args:
            host: Push server host address to connect to
            port: Push server port to connect to
            hwm: High water mark (max queued messages)
            context: Optional existing ZMQ context to use
        """
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.RCVHWM, hwm)
        self.socket.connect(f"tcp://{host}:{port}")

        # Handler registry
        self.handlers = {}

        # Default handler
        self.default_handler = None

        # Stats
        self.stats = {
            "received_items": 0,
            "processed_items": 0,
            "errors": 0,
            "last_receive_time": 0,
        }

    def register_handler(self, work_type: str, handler: Callable) -> None:
        """
        Register a handler for a specific work type.

        Args:
            work_type: The work type to handle
            handler: Async callback function that takes the work item as an argument
        """
        self.handlers[work_type] = handler

    def register_default_handler(self, handler: Callable) -> None:
        """
        Register a default handler for unknown work types.

        Args:
            handler: Async callback function that takes (work_type, work_item) as arguments
        """
        self.default_handler = handler

    async def start(self) -> None:
        """Start receiving work items."""
        logger.info(f"Starting PULL client to {self.host}:{self.port}")
        self.running = True
        # Store the task reference for proper shutdown
        self._receive_task = asyncio.create_task(self._receive_loop())

    async def stop(self) -> None:
        """Stop receiving work items with RTOS-friendly cleanup."""
        logger.info("Stopping PULL client")
        self.running = False

        # Cancel and wait for the receive task
        if hasattr(self, "_receive_task"):
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                logger.debug("Pull client receive task cancelled successfully")
            except Exception as e:
                logger.warning(f"Error cancelling pull client task: {e}")

        # Close socket
        if self.socket:
            try:
                self.socket.close()
            except Exception as e:
                logger.warning(f"Error closing pull client socket: {e}")

    async def _receive_loop(self) -> None:
        """Main loop for receiving work items and dispatching to handlers with RTOS-friendly error handling."""
        while self.running:
            try:
                multipart = await self.socket.recv_multipart()

                # Expecting [work_type, content_type, data]
                if len(multipart) < 3:
                    logger.error(f"Received malformed message: {multipart}")
                    self.stats["errors"] += 1
                    continue

                work_type = multipart[0].decode()
                content_type = multipart[1].decode()
                data = deserialize_message(multipart[2], content_type)

                # Update statistics
                self.stats["received_items"] += 1
                self.stats["last_receive_time"] = time.time()

                logger.debug(f"Received work item of type: {work_type}")

                # Dispatch to handler
                if work_type in self.handlers:
                    try:
                        await self.handlers[work_type](data)
                        self.stats["processed_items"] += 1
                    except Exception as e:
                        logger.error(f"Error in handler for work type {work_type}: {e}")
                        self.stats["errors"] += 1
                elif self.default_handler:
                    try:
                        await self.default_handler(work_type, data)
                        self.stats["processed_items"] += 1
                    except Exception as e:
                        logger.error(
                            f"Error in default handler for work type {work_type}: {e}"
                        )
                        self.stats["errors"] += 1
                else:
                    logger.warning(f"No handler for work type: {work_type}")

            except asyncio.CancelledError:
                logger.debug("Receive loop cancelled")
                break
            except Exception as e:
                logger.error(f"Error receiving work item: {e}")
                self.stats["errors"] += 1
                # RTOS-friendly: Check running state and use async sleep
                if not self.running:
                    break
                # Use asyncio.sleep for proper async error recovery
                try:
                    await asyncio.sleep(
                        0.1
                    )  # @architecture:acceptable - error recovery
                except asyncio.CancelledError:
                    # Handle cancellation during error recovery
                    logger.debug("Receive loop cancelled during error recovery")
                    break


class PushPullManager:
    """
    Manager class for coordinating Push and Pull operations.

    This class provides a unified interface for the FEAGI ZMQ server
    to manage PUSH/PULL patterns.
    """

    def __init__(
        self,
        core_api: CoreAPIService,
        host: str = "*",
        port: int = 5557,
        context: Optional[zmq.asyncio.Context] = None,
    ):
        """
        Initialize a new PushPull Manager.

        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to
            port: Port number to bind to
            context: Optional existing ZMQ context to use
        """
        self.context = context or zmq.asyncio.Context.instance()
        self._port = port
        self.push_server = PushServer(core_api, host, port, context=self.context)

    @property
    def port(self) -> int:
        """Get the port used by this manager's server."""
        return self._port

    async def start(self) -> None:
        """Start the PushPull manager."""
        await self.push_server.start()

    async def stop(self) -> None:
        """Stop the PushPull manager with RTOS-friendly error handling."""
        try:
            # Check if we're in an event loop context
            try:
                loop = asyncio.get_running_loop()
                # We're in a loop, proceed normally
                await self.push_server.stop()
            except RuntimeError:
                # No running loop, try to stop gracefully without await
                if hasattr(self.push_server, "running"):
                    self.push_server.running = False
                if hasattr(self.push_server, "socket") and self.push_server.socket:
                    try:
                        self.push_server.socket.close()
                    except Exception as e:
                        logger.warning(
                            f"Error closing push server socket during no-loop shutdown: {e}"
                        )
        except Exception as e:
            logger.warning(f"Error stopping PushPull manager: {e}")

    async def queue_work(self, work_type: str, data: Any, priority: int = 0) -> None:
        """
        Queue a work item for processing.

        Args:
            work_type: Type of work item
            data: Work item data
            priority: Priority level (higher is processed first)
        """
        await self.push_server.push_item(data, work_type, priority)
