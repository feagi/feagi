"""
ZeroMQ Server for FEAGI API

This module implements the ZeroMQ server for the FEAGI API, providing
high-performance, real-time communication with clients.
"""

import asyncio
import logging
import time
from typing import Dict, Any, Optional, List, Tuple, Union

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

    async def start(self) -> None:
        """Start the ZMQ server."""
        logger.info(f"Starting ZMQ server on {self.host}")
        self.running = True
        
        # Start all servers
        await asyncio.gather(
            self.req_rep.start(),
            self.pub_sub.start(),
            self.push_pull.start(),
            self.sensorimotor.start(),
            self.visualization.start(),
            self._publish_system_metrics()
        )

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