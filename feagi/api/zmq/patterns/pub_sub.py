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
ZeroMQ Publisher-Subscriber Implementation for FEAGI API

This module implements the PUB/SUB pattern for ZeroMQ communication in FEAGI.
It provides:
- Publisher implementation for broadcasting events and updates
- Subscriber interfaces for receiving broadcast messages
- Topic-based filtering for selective message reception
"""

import asyncio
import json
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
import zmq
import zmq.asyncio
from typing import Dict, Any, List, Callable, Optional, Union, Callable

from ...core.services.core_api_service import CoreAPIService
from ..serialization import serialize_message, deserialize_message
from ...utils.auth import validate_token
from ...utils.rate_limit import RateLimiter


class PublisherServer:
    """
    ZeroMQ Publisher server implementation.
    
    This server broadcasts messages to multiple subscribers using the PUB/SUB pattern.
    It's designed for one-to-many communication where subscribers can filter 
    messages based on topics.
    """
    
    def __init__(
        self, 
        core_api: CoreAPIService,
        host: str = "*", 
        port: int = 5556,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize a new Publisher server.
        
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
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{host}:{port}")
        self.rate_limiter = RateLimiter()
        
        # Topic handlers map
        self.topics = {
            "brain.activity": self._handle_brain_activity,
            "simulation.status": self._handle_simulation_status,
            "stats.performance": self._handle_performance_stats,
            "system.events": self._handle_system_events,
            "monitoring.logs": self._handle_log_events
        }
        
        # Keep periodic task references
        self.periodic_tasks = {}

    async def start(self) -> None:
        """Start the publisher server and initialize periodic broadcasting."""
        logger.info(f"Starting PUB server on {self.host}:{self.port}")
        self.running = True
        
        # Store the current event loop for this method
        self._event_loop = asyncio.get_event_loop()
        
        # Start periodic broadcasting tasks in the current loop
        self.periodic_tasks["simulation_status"] = self._event_loop.create_task(
            self._broadcast_simulation_status()
        )
        self.periodic_tasks["performance_stats"] = self._event_loop.create_task(
            self._broadcast_performance_stats()
        )

    async def stop(self) -> None:
        """Stop the publisher server and all periodic tasks."""
        logger.info("Stopping PUB server")
        self.running = False
        
        # Cancel all periodic tasks
        for task_name, task in self.periodic_tasks.items():
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    logger.debug(f"Cancelled periodic task: {task_name}")
        
        self.socket.close()

    async def publish(
        self, 
        topic: str, 
        data: Any, 
        content_type: str = "application/json"
    ) -> None:
        """
        Publish a message to a specific topic.
        
        Args:
            topic: The topic string to publish to
            data: The data to publish
            content_type: Content type for serialization (default: application/json)
        """
        serialized_data = serialize_message(data, content_type)
        await self.socket.send_multipart([
            topic.encode(),
            content_type.encode(),
            serialized_data
        ])
        logger.debug(f"Published message to topic: {topic}")

    async def _broadcast_simulation_status(self) -> None:
        """Periodically broadcast simulation status updates."""
        while self.running:
            try:
                status = await self.core_api.get_simulation_status()
                await self.publish("simulation.status", status)
            except asyncio.CancelledError:
                logger.debug("Simulation status broadcast cancelled")
                break
            except Exception as e:
                logger.error(f"Error broadcasting simulation status: {e}")
            
            # RTOS-friendly: Use cancellable sleep
            try:
                await asyncio.sleep(1.0)  # Update every second
            except asyncio.CancelledError:
                logger.debug("Simulation status broadcast cancelled during sleep")
                break

    async def _broadcast_performance_stats(self) -> None:
        """Periodically broadcast performance statistics."""
        while self.running:
            try:
                stats = await self.core_api.get_performance_stats()
                await self.publish("stats.performance", stats)
            except asyncio.CancelledError:
                logger.debug("Performance stats broadcast cancelled")
                break
            except Exception as e:
                logger.error(f"Error broadcasting performance stats: {e}")
            
            # RTOS-friendly: Use cancellable sleep
            try:
                await asyncio.sleep(5.0)  # Update every 5 seconds
            except asyncio.CancelledError:
                logger.debug("Performance stats broadcast cancelled during sleep")
                break

    async def _handle_brain_activity(self) -> Dict:
        """Get current brain activity data for broadcasting."""
        # This would typically fetch current firing data from the FCL Manager
        # For now, return a default empty structure
        return {"activity": [], "timestamp": time.time()}

    async def _handle_simulation_status(self) -> Dict:
        """Get current simulation status for broadcasting."""
        return await self.core_api.get_simulation_status()

    async def _handle_performance_stats(self) -> Dict:
        """Get performance statistics for broadcasting."""
        return await self.core_api.get_performance_stats()

    async def _handle_system_events(self) -> Dict:
        """Get system events for broadcasting."""
        # Instead of trying to call a non-existent method, return a default empty structure
        return {"events": [], "timestamp": time.time()}

    async def _handle_log_events(self) -> Dict:
        """Get log events for broadcasting."""
        # Instead of trying to call a non-existent method, return a default empty structure
        return {"logs": [], "timestamp": time.time()}

    async def broadcast_event(self, event_type: str, event_data: Dict) -> None:
        """
        Broadcast a system event message.
        
        Args:
            event_type: Type of the event (e.g., "cortical_area.created")
            event_data: Event data payload
        """
        message = {
            "type": event_type,
            "timestamp": time.time(),
            "data": event_data
        }
        await self.publish("system.events", message)


class SubscriberClient:
    """
    ZeroMQ Subscriber client implementation.
    
    This client connects to a Publisher and receives messages based on subscribed topics.
    """
    
    def __init__(
        self, 
        host: str,  # Remove hardcoded default - must be provided from configuration
        port: int = 5556,
        topics: Optional[List[str]] = None,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize a new Subscriber client.
        
        Args:
            host: Publisher host address to connect to
            port: Publisher port to connect to
            topics: List of topics to subscribe to (default: subscribe to all)
            context: Optional existing ZMQ context to use
        """
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{host}:{port}")
        
        # Set topic filters
        if topics:
            for topic in topics:
                self.socket.setsockopt(zmq.SUBSCRIBE, topic.encode())
        else:
            # Subscribe to all messages
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        
        # Callback registry
        self.callbacks = {}

    def register_callback(self, topic: str, callback: Callable) -> None:
        """
        Register a callback for a specific topic.
        
        Args:
            topic: The topic to register for
            callback: The callback function to invoke when a message is received
        """
        self.callbacks[topic] = callback
        # Ensure we're subscribed to this topic
        self.socket.setsockopt(zmq.SUBSCRIBE, topic.encode())

    def unregister_callback(self, topic: str) -> None:
        """
        Unregister a callback for a specific topic.
        
        Args:
            topic: The topic to unregister
        """
        if topic in self.callbacks:
            del self.callbacks[topic]
            # Unsubscribe if no callbacks remain for this topic
            if topic not in self.callbacks:
                self.socket.setsockopt(zmq.UNSUBSCRIBE, topic.encode())

    async def start(self) -> None:
        """Start receiving messages."""
        logger.info(f"Starting SUB client to {self.host}:{self.port}")
        self.running = True
        asyncio.create_task(self._receive_loop())

    async def stop(self) -> None:
        """Stop receiving messages."""
        logger.info("Stopping SUB client")
        self.running = False
        self.socket.close()

    async def _receive_loop(self) -> None:
        """Main loop for receiving messages and dispatching to callbacks."""
        while self.running:
            try:
                multipart = await self.socket.recv_multipart()
                
                # Expecting [topic, content_type, data]
                if len(multipart) < 3:
                    logger.error(f"Received malformed message: {multipart}")
                    continue
                
                topic = multipart[0].decode()
                content_type = multipart[1].decode()
                data = deserialize_message(multipart[2], content_type)
                
                logger.debug(f"Received message on topic: {topic}")
                
                # Invoke registered callbacks
                if topic in self.callbacks:
                    try:
                        await self.callbacks[topic](data)
                    except Exception as e:
                        logger.error(f"Error in callback for topic {topic}: {e}")
            
            except asyncio.CancelledError:
                logger.debug("Receive loop cancelled")
                break
            except Exception as e:
                logger.error(f"Error receiving message: {e}")
                # RTOS-friendly: Use cancellable sleep for error recovery
                try:
                    await asyncio.sleep(1)  # Avoid tight loop on errors
                except asyncio.CancelledError:
                    logger.debug("Receive loop cancelled during error recovery")
                    break


class PubSubManager:
    """
    Manager class for coordinating Publishers and Subscribers.
    
    This class provides a unified interface for the FEAGI ZMQ server
    to manage PUB/SUB patterns.
    """
    
    def __init__(
        self, 
        core_api: CoreAPIService,
        host: str = "*", 
        port: int = 5556,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize a new PubSub Manager.
        
        Args:
            core_api: The CoreAPIService instance to delegate calls to
            host: Host address to bind to
            port: Port number to bind to
            context: Optional existing ZMQ context to use
        """
        self.context = context or zmq.asyncio.Context.instance()
        self._port = port
        self.publisher = PublisherServer(core_api, host, port, self.context)
    
    @property
    def port(self) -> int:
        """Get the port used by this manager's server."""
        return self._port
    
    async def start(self) -> None:
        """Start the PubSub manager."""
        await self.publisher.start()
    
    async def stop(self) -> None:
        """Stop the PubSub manager."""
        await self.publisher.stop()
    
    async def publish_event(self, event_type: str, event_data: Dict) -> None:
        """
        Publish an event to all subscribers.
        
        Args:
            event_type: Type of the event
            event_data: Event data
        """
        await self.publisher.broadcast_event(event_type, event_data) 