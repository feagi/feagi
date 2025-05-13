"""
ZeroMQ Server for FEAGI API

This module implements the ZeroMQ server for the FEAGI API, providing
high-performance, real-time communication with clients.
"""

import os
import time
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import threading
import asyncio
import concurrent.futures
from typing import Dict, Any, List, Optional, Union, Callable

import zmq
import zmq.asyncio
from zmq.auth.thread import ThreadAuthenticator

from ..core.service import CoreApiService


class ZmqServer:
    """
    ZMQ server for FEAGI with proper event loop management.
    
    This implementation ensures each thread has its own event loop and
    properly manages asyncio resources.
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
        Initialize the ZMQ server.
        
        Args:
            core_api: Core API service for accessing FEAGI
            host: Host to bind to
            req_rep_port: Port for REQ/REP pattern
            pub_sub_port: Port for PUB/SUB pattern
            push_pull_port: Port for PUSH/PULL pattern
            sensorimotor_port: Port for sensorimotor stream
            vis_base_port: Base port for visualization stream
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.req_rep_port = req_rep_port
        self.pub_sub_port = pub_sub_port
        self.push_pull_port = push_pull_port
        self.sensorimotor_port = sensorimotor_port
        self.vis_base_port = vis_base_port
        
        # Thread and event loop management
        self._thread = None
        self._loop = None
        self._context = context or zmq.asyncio.Context.instance()
        self._running = False
        self._shutdown_event = threading.Event()
        
        # Pattern managers
        self._req_rep = None
        self._pub_sub = None
        self._push_pull = None
        self._sensorimotor = None
        self._visualization = None
    
    def start(self) -> bool:
        """
        Start the ZMQ server in a background thread.
        
        Returns:
            True if started successfully, False otherwise
        """
        if self._running:
            logger.warning("ZMQ server is already running")
            return True
        
        logger.info(f"Starting ZMQ server on {self.host}")
        try:
            # In synchronous mode, create a context and start the server in a background thread
            logger.info("Creating background thread for ZMQ server")
            self._thread = threading.Thread(target=self._run_server_thread, daemon=True)
            self._thread.start()
            
            # Wait briefly to allow the server to start or fail
            time.sleep(0.5)
            
            if not self._running:
                # If the server didn't start properly, the thread will have set _running to False
                logger.error("ZMQ server failed to start")
                return False
                
            return True
        except Exception as e:
            logger.error(f"Error starting ZMQ server: {e}")
            if self._context:
                self._context.term()
                self._context = None
            return False
    
    def _run_server_thread(self):
        """
        Run the ZMQ server in a background thread.
        
        This method creates a new event loop for the thread and runs the server in it.
        """
        # Create a new event loop for this thread
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            # Initialize and run the server
            self._running = True
            self._loop.run_until_complete(self._start_server())
            self._loop.run_until_complete(self._monitor_loop())
        except Exception as e:
            logger.error(f"Error in ZMQ server thread: {e}")
            self._running = False
        finally:
            self._cleanup()
    
    async def _start_server(self):
        """
        Start all ZMQ services asynchronously.
        
        This is called from the background thread.
        """
        try:
            # We'll import here to avoid circular imports
            from .patterns.req_rep import RequestReplyManager
            from .patterns.pub_sub import PubSubManager
            from .patterns.push_pull import PushPullManager
            from .streams.sensorimotor import SensorimotorStream
            from .streams.visualization import VisualizationStream
            
            # Initialize all managers with the current thread's event loop
            self._req_rep = RequestReplyManager(
                core_api=self.core_api,
                host=self.host,
                port=self.req_rep_port,
                context=self._context
            )
            
            self._pub_sub = PubSubManager(
                core_api=self.core_api,
                host=self.host,
                port=self.pub_sub_port,
                context=self._context
            )
            
            self._push_pull = PushPullManager(
                core_api=self.core_api,
                host=self.host,
                port=self.push_pull_port,
                context=self._context
            )
            
            self._sensorimotor = SensorimotorStream(
                core_api=self.core_api,
                host=self.host,
                port=self.sensorimotor_port,
                context=self._context
            )
            
            self._visualization = VisualizationStream(
                core_api=self.core_api,
                host=self.host,
                structure_port=self.vis_base_port,
                activity_port=self.vis_base_port + 1,
                control_port=self.vis_base_port + 2,
                context=self._context
            )
            
            # Start all managers
            await self._req_rep.start()
            await self._pub_sub.start()
            await self._push_pull.start()
            await self._sensorimotor.start()
            await self._visualization.start()
            
            logger.info("ZMQ server started successfully")
        except Exception as e:
            logger.error(f"Failed to start ZMQ services: {e}")
            self._running = False
            raise
    
    async def _monitor_loop(self):
        """
        Monitor loop to keep the server running and handle shutdown requests.
        """
        while self._running and not self._shutdown_event.is_set():
            try:
                # Publish system metrics periodically
                try:
                    if self.core_api and self._pub_sub:
                        metrics = await self.core_api.get_system_metrics()
                        if metrics:
                            await self._pub_sub.publish_event("system.metrics", metrics)
                except Exception as e:
                    logger.error(f"Error publishing system metrics: {e}")
                
                # Sleep a bit to avoid high CPU usage
                await asyncio.sleep(1.0)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in ZMQ monitor loop: {e}")
                await asyncio.sleep(1.0)
    
    def shutdown(self):
        """
        Shut down the ZMQ server.
        
        This method can be called from any thread and will properly signal
        the server thread to shut down and clean up resources.
        """
        logger.info("Shutting down ZMQ server")
        if not self._running:
            logger.warning("ZMQ server is not running, nothing to shut down")
            return
        
        self._running = False
        self._shutdown_event.set()
        
        if self._thread and self._thread.is_alive():
            logger.info("Signaling server thread to stop")
            if self._loop:
                # Create a future to run the stop() coroutine in the server thread's event loop
                future = asyncio.run_coroutine_threadsafe(self._stop_services(), self._loop)
                try:
                    # Wait for the stop to complete with a timeout
                    future.result(timeout=5.0)
                except concurrent.futures.TimeoutError:
                    logger.warning("Timeout waiting for ZMQ server to stop")
                except Exception as e:
                    logger.error(f"Error stopping ZMQ server: {e}")
            
            # Give the thread some time to clean up
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                logger.warning("ZMQ server thread did not exit cleanly")
        
        logger.info("ZMQ server shutdown complete")
    
    async def _stop_services(self):
        """
        Stop all ZMQ services.
        
        This coroutine is run in the server thread's event loop.
        """
        logger.info("Stopping ZMQ services...")
        try:
            # Stop all services
            stop_tasks = []
            
            if self._req_rep:
                stop_tasks.append(self._req_rep.stop())
            if self._pub_sub:
                stop_tasks.append(self._pub_sub.stop())
            if self._push_pull:
                stop_tasks.append(self._push_pull.stop())
            if self._sensorimotor:
                stop_tasks.append(self._sensorimotor.stop())
            if self._visualization:
                stop_tasks.append(self._visualization.stop())
            
            if stop_tasks:
                await asyncio.gather(*stop_tasks, return_exceptions=True)
                
            logger.info("All ZMQ services stopped")
        except Exception as e:
            logger.error(f"Error stopping ZMQ services: {e}")
    
    def _cleanup(self):
        """
        Clean up resources after shutdown.
        
        This method is called from the server thread.
        """
        logger.info("Cleaning up ZMQ resources")
        try:
            # Close the event loop
            if self._loop and not self._loop.is_closed():
                pending_tasks = asyncio.all_tasks(self._loop)
                if pending_tasks:
                    logger.warning(f"Cancelling {len(pending_tasks)} pending tasks")
                    for task in pending_tasks:
                        task.cancel()
                    
                    # Give tasks a chance to cancel
                    self._loop.run_until_complete(
                        asyncio.gather(*pending_tasks, return_exceptions=True)
                    )
                
                self._loop.close()
                logger.info("Event loop closed")
            
            # Term the context
            if self._context:
                self._context.term()
                logger.info("ZMQ context terminated")
                self._context = None
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
    
    async def publish_event(self, event_type: str, event_data: Dict) -> None:
        """
        Publish an event to subscribers.
        
        Args:
            event_type: The type of event
            event_data: The event data
        """
        if not self._running:
            logger.warning(f"ZMQ server not running, can't publish event: {event_type}")
            return
            
        if not self._pub_sub:
            logger.warning(f"PubSub manager not initialized, can't publish event: {event_type}")
            return
            
        try:
            # We need to schedule this in the server thread's event loop
            if self._loop:
                future = asyncio.run_coroutine_threadsafe(
                    self._pub_sub.publish_event(event_type, event_data), 
                    self._loop
                )
                future.result(timeout=1.0)  # Wait with timeout to avoid blocking
            else:
                logger.warning(f"No event loop available, can't publish event: {event_type}")
        except Exception as e:
            logger.error(f"Error publishing event {event_type}: {e}")
    
    async def queue_work(self, work_type: str, data: Any, priority: int = 0) -> None:
        """
        Queue work for processing.
        
        Args:
            work_type: The type of work
            data: The work data
            priority: Work priority (0-9, higher is more important)
        """
        if not self._running:
            logger.warning(f"ZMQ server not running, can't queue work: {work_type}")
            return
            
        if not self._push_pull:
            logger.warning(f"PushPull manager not initialized, can't queue work: {work_type}")
            return
            
        try:
            # We need to schedule this in the server thread's event loop
            if self._loop:
                future = asyncio.run_coroutine_threadsafe(
                    self._push_pull.queue_work(work_type, data, priority), 
                    self._loop
                )
                future.result(timeout=1.0)  # Wait with timeout to avoid blocking
            else:
                logger.warning(f"No event loop available, can't queue work: {work_type}")
        except Exception as e:
            logger.error(f"Error queueing work {work_type}: {e}") 