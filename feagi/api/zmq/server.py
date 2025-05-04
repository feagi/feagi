"""
ZeroMQ Server for FEAGI API

This module implements the ZeroMQ server for the FEAGI API, providing
high-performance, real-time communication with clients.
"""

import asyncio
import logging
import time
from typing import Dict, Any, Optional, List, Tuple, Union
import concurrent.futures

import zmq
import zmq.asyncio

from ..core.service import CoreApiService
from .patterns.req_rep import RequestReplyManager
from .patterns.pub_sub import PubSubManager
from .patterns.push_pull import PushPullManager
from .streams.sensorimotor import SensorimotorStream
from .streams.visualization import VisualizationStream

logger = logging.getLogger(__name__)


class ZmqServer:
    """
    ZeroMQ server implementation for FEAGI API.
    
    This server integrates multiple ZeroMQ patterns and streams to provide
    a comprehensive API for FEAGI clients:
    
    - Request-Reply: For traditional CRUD operations
    - Publish-Subscribe: For broadcasting events and updates
    - Push-Pull: For high-throughput data processing
    - Sensorimotor Stream: For efficient binary exchange of sensorimotor data
    - Visualization Stream: For brain activity visualization
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*",
        req_rep_port: int = 5555,
        pub_sub_port: int = 5556,
        push_pull_port: int = 5557,
        sensorimotor_port: int = 5558,
        vis_base_port: int = 5560,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize a new ZMQ server.
        
        Args:
            core_api: The CoreApiService instance to delegate calls to
            host: Host address to bind to
            req_rep_port: Port for Request-Reply pattern
            pub_sub_port: Port for Publish-Subscribe pattern
            push_pull_port: Port for Push-Pull pattern
            sensorimotor_port: Port for Sensorimotor stream
            vis_base_port: Base port for Visualization stream
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.context = context or zmq.asyncio.Context.instance()
        
        # Initialize pattern managers
        self.req_rep = RequestReplyManager(
            core_api=core_api,
            host=host,
            port=req_rep_port,
            context=self.context
        )
        
        self.pub_sub = PubSubManager(
            core_api=core_api,
            host=host,
            port=pub_sub_port,
            context=self.context
        )
        
        self.push_pull = PushPullManager(
            core_api=core_api,
            host=host,
            port=push_pull_port,
            context=self.context
        )
        
        # Initialize specialized streams
        self.sensorimotor = SensorimotorStream(
            core_api=core_api,
            host=host,
            port=sensorimotor_port,
            context=self.context
        )
        
        self.visualization = VisualizationStream(
            core_api=core_api,
            host=host,
            structure_port=vis_base_port,
            activity_port=vis_base_port + 1,
            control_port=vis_base_port + 2,
            context=self.context
        )
        
        # Periodically publish system metrics
        self.running = False

    async def async_start(self) -> None:
        """Start the ZMQ server asynchronously."""
        logger.info(f"Starting ZMQ server on {self.host}")
        self.running = True
        
        # Store the current event loop
        self._event_loop = asyncio.get_event_loop()
        
        # Start all servers
        await asyncio.gather(
            self.req_rep.start(),
            self.pub_sub.start(),
            self.push_pull.start(),
            self.sensorimotor.start(),
            self.visualization.start()
        )
        
        # Start the system metrics publisher
        self._metrics_task = self._event_loop.create_task(self._publish_system_metrics())

    def start(self) -> bool:
        """
        Synchronous method to start the ZMQ server.
        
        Returns:
            bool: True if the server started successfully, False otherwise.
        """
        logger.info(f"Starting ZMQ server on {self.host} (sync)")
        
        try:
            # Create a new event loop if needed
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                # No event loop in current thread
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
            
            # For the synchronous version, we'll just flag the server as running
            # and initialize the components, but we won't block waiting for them
            self.running = True
            
            # Create a thread-local store for the event loop
            if not hasattr(self, '_thread_local'):
                import threading
                self._thread_local = threading.local()
            self._thread_local.loop = loop
                
            # Start the async components in a background task if the loop is running
            if loop.is_running():
                logger.info("Event loop is running, using create_task for startup")
                future = asyncio.run_coroutine_threadsafe(self.async_start(), loop)
                # Wait briefly to catch immediate errors
                try:
                    future.result(0.1)
                except concurrent.futures.TimeoutError:
                    pass  # This is expected, async_start doesn't complete quickly
            else:
                # If the loop isn't running, we need to create a new thread to run it
                logger.info("Creating background thread for ZMQ server")
                import threading
                def run_server():
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    if not hasattr(self, '_thread_local'):
                        self._thread_local = threading.local()
                    self._thread_local.loop = loop
                    loop.run_until_complete(self.async_start())
                    loop.run_forever()
                
                self._thread = threading.Thread(target=run_server, daemon=True)
                self._thread.start()
            
            return True
        except Exception as e:
            logger.error(f"Error starting ZMQ server: {e}")
            self.running = False
            return False

    async def stop(self) -> None:
        """Stop the ZMQ server."""
        logger.info("Stopping ZMQ server")
        self.running = False
        
        # Stop all servers
        await asyncio.gather(
            self.req_rep.stop(),
            self.pub_sub.stop(),
            self.push_pull.stop(),
            self.sensorimotor.stop(),
            self.visualization.stop()
        )
        
        # Close context
        self.context.term()

    async def _publish_system_metrics(self) -> None:
        """Periodically publish system metrics."""
        while self.running:
            try:
                # Gather system metrics
                metrics = await self.core_api.get_system_metrics()
                
                # Publish to subscribers
                await self.pub_sub.publish_event("system.metrics", metrics)
                
                # Publish metrics every 5 seconds
                await asyncio.sleep(5.0)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error publishing system metrics: {e}")
                await asyncio.sleep(5.0)

    def shutdown(self) -> None:
        """
        Synchronous method to shutdown the ZMQ server.
        
        This is a non-async wrapper around the async stop() method,
        intended for use in synchronous code like the main process.
        """
        logger.info("Shutting down ZMQ server (sync)")
        self.running = False
        
        # Get the event loop from thread local storage if it exists
        loop = getattr(self._thread_local, 'loop', None) if hasattr(self, '_thread_local') else None
        
        # If we don't have a stored loop, try to get the current one
        if loop is None:
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                # No event loop in current thread
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
        
        # Run the stop coroutine
        if not loop.is_closed():
            try:
                if loop.is_running():
                    logger.info("Event loop is running, using create_task for shutdown")
                    future = asyncio.run_coroutine_threadsafe(self.stop(), loop)
                    # Wait a bit for shutdown to proceed
                    try:
                        future.result(1.0)
                    except concurrent.futures.TimeoutError:
                        logger.warning("Shutdown taking longer than expected")
                else:
                    logger.info("Running stop() in event loop")
                    loop.run_until_complete(self.stop())
            except Exception as e:
                logger.error(f"Error during ZMQ server shutdown: {e}")
        else:
            logger.warning("Event loop is closed, cannot properly shutdown ZMQ server")
            
        # Make sure we terminate the ZMQ context
        try:
            # Try to close the context directly
            if hasattr(self, 'context') and self.context:
                try:
                    self.context.term()
                except Exception as e:
                    logger.error(f"Error terminating ZMQ context: {e}")
        except Exception as e:
            logger.error(f"Error during final cleanup: {e}")

    async def publish_event(self, event_type: str, event_data: Dict) -> None:
        """
        Publish an event to all subscribers.
        
        Args:
            event_type: Type of the event
            event_data: Event data
        """
        await self.pub_sub.publish_event(event_type, event_data)

    async def queue_work(self, work_type: str, data: Any, priority: int = 0) -> None:
        """
        Queue a work item for processing.
        
        Args:
            work_type: Type of work item
            data: Work item data
            priority: Priority level (higher is processed first)
        """
        await self.push_pull.queue_work(work_type, data, priority) 