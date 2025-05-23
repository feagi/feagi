"""
ZeroMQ REST Stream Implementation for FEAGI API

This module implements a dedicated REST API stream for FEAGI.
It provides:
- Pure REST API protocol handling over ZMQ
- HTTP-like request/response semantics over ZMQ transport
- JSON-based REST message format
- ROUTER-DEALER pattern for scalable request handling

The REST stream is designed specifically for REST API operations:
- GET, POST, PUT, DELETE operations
- Standard HTTP status codes and headers
- JSON request/response bodies
- Route-based request handling
- Stateless operation model

This stream handles ONLY REST format messages and does not provide
backward compatibility with legacy control message formats.
"""

import asyncio
import json
import time
import uuid
from typing import Dict, Any, Optional

import zmq
import zmq.asyncio

from feagi.utils.logger import setup_logger
from ...core.service import CoreApiService
from ..rest_adapter import ZMQRestAPIAdapter

logger = setup_logger(__name__)


class RestStream:
    """
    ZeroMQ REST Stream implementation for pure REST API operations.
    
    This implementation provides a dedicated endpoint for REST API requests
    using a ROUTER-DEALER pattern for scalable, stateless operation.
    
    The REST stream handles ONLY:
    - REST API requests with 'method' and 'route' fields
    - Standard HTTP semantics (GET, POST, PUT, DELETE)
    - JSON request/response format
    - HTTP-like status codes and headers
    
    This stream does NOT handle legacy control messages or other protocols.
    """
    
    def __init__(
        self,
        core_api: CoreApiService,
        host: str = "*",
        port: int = 5563,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize the REST stream.
        
        Args:
            core_api: Core API service for accessing FEAGI
            host: Host to bind to
            port: Port for the REST socket
            context: Optional existing ZMQ context to use
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.context = context or zmq.asyncio.Context.instance()
        
        # State
        self.server_id = f"feagi_rest_server_{uuid.uuid4().hex[:8]}"
        self.running = False
        
        # Statistics
        self.stats = {
            'requests_processed': 0,
            'requests_success': 0,
            'requests_error': 0,
            'start_time': None
        }
        
        # Initialize sockets
        self.router_socket = None  # Front-facing socket for external clients
        self.dealer_socket = None  # Back-end socket for internal processing
        
        # Tasks
        self.tasks = []
        
        # REST API adapter for handling REST format messages
        self.rest_adapter = ZMQRestAPIAdapter(core_api)
        
        logger.info(f"REST stream initialized for {host}:{port}")
        
    async def start(self):
        """Start the REST stream."""
        if self.running:
            logger.warning("REST stream is already running")
            return
            
        logger.info(f"Starting REST Stream on {self.host}:{self.port}")
        
        try:
            # Create ROUTER socket (for external clients)
            self.router_socket = self.context.socket(zmq.ROUTER)
            self.router_socket.bind(f"tcp://{self.host}:{self.port}")
            
            # Create DEALER socket (for internal routing)
            self.dealer_socket = self.context.socket(zmq.DEALER)
            self.dealer_socket.bind("inproc://rest_backend")
            
            # Start worker threads
            self.tasks.append(asyncio.create_task(self._router_dealer_proxy()))
            self.tasks.append(asyncio.create_task(self._process_rest_messages()))
            
            # Start statistics tracking
            self.stats['start_time'] = time.time()
            
            self.running = True
            logger.info("✅ REST Stream started successfully")
            
        except Exception as e:
            logger.error(f"❌ Failed to start REST stream: {e}")
            await self.stop()
            raise
        
    async def stop(self):
        """Stop the REST stream."""
        if not self.running:
            return
            
        logger.info("Stopping REST Stream")
        
        # Cancel all tasks
        for task in self.tasks:
            task.cancel()
            
        # Wait for tasks to complete
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
            self.tasks = []
        
        # Close sockets
        if self.router_socket:
            self.router_socket.close(linger=0)
            self.router_socket = None
            
        if self.dealer_socket:
            self.dealer_socket.close(linger=0)
            self.dealer_socket = None
            
        self.running = False
        
        # Print final statistics
        self._log_statistics(final=True)
        logger.info("🛑 REST Stream stopped")
        
    async def _router_dealer_proxy(self):
        """
        Run a ROUTER-DEALER proxy to route messages between external clients and internal workers.
        """
        logger.debug("Starting ROUTER-DEALER proxy for REST stream")
        
        try:
            # Create a poller for the sockets
            poller = zmq.asyncio.Poller()
            poller.register(self.router_socket, zmq.POLLIN)
            poller.register(self.dealer_socket, zmq.POLLIN)
            
            while self.running:
                try:
                    events = dict(await poller.poll(timeout=1000))
                    
                    # Forward messages from router to dealer
                    if self.router_socket in events:
                        message = await self.router_socket.recv_multipart()
                        await self.dealer_socket.send_multipart(message)
                        
                    # Forward messages from dealer to router
                    if self.dealer_socket in events:
                        message = await self.dealer_socket.recv_multipart()
                        await self.router_socket.send_multipart(message)
                        
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error in REST ROUTER-DEALER proxy: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Fatal error in REST ROUTER-DEALER proxy: {e}")
        finally:
            logger.debug("REST ROUTER-DEALER proxy stopped")
            
    async def _process_rest_messages(self):
        """Process REST messages from the dealer socket."""
        logger.debug("Starting REST message processor")
        
        # Create a worker socket to connect to the dealer
        worker_socket = self.context.socket(zmq.DEALER)
        worker_socket.connect("inproc://rest_backend")
        
        try:
            while self.running:
                try:
                    # Receive message with timeout
                    message_parts = await worker_socket.recv_multipart()
                    
                    # Message format: [client_id, empty, message]
                    if len(message_parts) < 3:
                        logger.warning(f"Invalid REST message format: {len(message_parts)} parts")
                        self.stats['requests_error'] += 1
                        continue
                        
                    client_id = message_parts[0]
                    message_data = message_parts[2]
                    
                    self.stats['requests_processed'] += 1
                    
                    # Try to decode as JSON
                    try:
                        message = json.loads(message_data.decode('utf-8'))
                    except json.JSONDecodeError as e:
                        logger.error(f"Invalid JSON in REST message: {e}")
                        
                        # Send error response
                        error_response = {
                            "status": 400,
                            "headers": {"content-type": "application/json"},
                            "body": {
                                "type": "error", 
                                "code": "INVALID_JSON", 
                                "message": f"Invalid JSON format: {str(e)}"
                            },
                            "timestamp": int(time.time() * 1000)
                        }
                        
                        await worker_socket.send_multipart([
                            client_id, 
                            b'', 
                            json.dumps(error_response).encode('utf-8')
                        ])
                        
                        self.stats['requests_error'] += 1
                        continue
                    
                    # Validate REST format
                    if not self._is_valid_rest_message(message):
                        logger.error(f"Invalid REST message format: {message}")
                        
                        error_response = {
                            "status": 400,
                            "headers": {"content-type": "application/json"},
                            "body": {
                                "type": "error",
                                "code": "INVALID_REST_FORMAT",
                                "message": "Message must contain 'method' and 'route' fields"
                            },
                            "timestamp": int(time.time() * 1000)
                        }
                        
                        await worker_socket.send_multipart([
                            client_id,
                            b'',
                            json.dumps(error_response).encode('utf-8')
                        ])
                        
                        self.stats['requests_error'] += 1
                        continue
                    
                    # Process with REST API adapter
                    method = message.get('method', 'UNKNOWN')
                    route = message.get('route', 'unknown')
                    
                    logger.debug(f"Processing REST API request: {method} {route}")
                    
                    try:
                        response_data = await self.rest_adapter.process_message(message_data)
                        
                        # Send response
                        await worker_socket.send_multipart([
                            client_id,
                            b'',
                            response_data
                        ])
                        
                        self.stats['requests_success'] += 1
                        logger.debug(f"✅ REST request completed: {method} {route}")
                        
                    except Exception as e:
                        logger.error(f"❌ Error processing REST request {method} {route}: {e}")
                        
                        # Send internal server error
                        error_response = {
                            "status": 500,
                            "headers": {"content-type": "application/json"},
                            "body": {
                                "type": "error",
                                "code": "INTERNAL_SERVER_ERROR",
                                "message": f"Internal server error: {str(e)}"
                            },
                            "timestamp": int(time.time() * 1000)
                        }
                        
                        await worker_socket.send_multipart([
                            client_id,
                            b'',
                            json.dumps(error_response).encode('utf-8')
                        ])
                        
                        self.stats['requests_error'] += 1
                        
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error processing REST message: {e}")
                    self.stats['requests_error'] += 1
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Fatal error in REST message processor: {e}")
        finally:
            # Clean up worker socket
            worker_socket.close()
            logger.debug("REST message processor stopped")
    
    def _is_valid_rest_message(self, message: Dict[str, Any]) -> bool:
        """
        Validate that a message is a valid REST format.
        
        Args:
            message: Decoded JSON message
            
        Returns:
            True if valid REST format, False otherwise
        """
        return (
            isinstance(message, dict) and
            'method' in message and 
            'route' in message and
            isinstance(message['method'], str) and
            isinstance(message['route'], str)
        )
    
    def _log_statistics(self, final: bool = False):
        """Log REST stream statistics."""
        if self.stats['start_time'] is None:
            return
            
        uptime = time.time() - self.stats['start_time']
        total_requests = self.stats['requests_processed']
        success_rate = (self.stats['requests_success'] / total_requests * 100) if total_requests > 0 else 100
        requests_per_second = total_requests / uptime if uptime > 0 else 0
        
        prefix = "🏁 Final" if final else "📊"
        
        logger.info(f"{prefix} REST Stream Statistics:")
        logger.info(f"   ⏱️  Uptime: {uptime:.1f}s")
        logger.info(f"   📊 Total requests: {total_requests}")
        logger.info(f"   ✅ Success: {self.stats['requests_success']}")
        logger.info(f"   ❌ Errors: {self.stats['requests_error']}")
        logger.info(f"   📈 Success rate: {success_rate:.1f}%")
        logger.info(f"   🚀 Requests/sec: {requests_per_second:.2f}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get current REST stream statistics."""
        uptime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        total_requests = self.stats['requests_processed']
        
        return {
            'uptime_seconds': uptime,
            'total_requests': total_requests,
            'successful_requests': self.stats['requests_success'],
            'error_requests': self.stats['requests_error'],
            'success_rate_percent': (self.stats['requests_success'] / total_requests * 100) if total_requests > 0 else 100,
            'requests_per_second': total_requests / uptime if uptime > 0 else 0,
            'running': self.running
        } 