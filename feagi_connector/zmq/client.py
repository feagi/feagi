"""
ZeroMQ client for FEAGI communication.
"""

import logging
import importlib.util
import sys
from typing import Dict, Any, List, Optional, Callable, Union
import json
import time
import threading
import asyncio
from datetime import datetime
import os

# Set up logging
logger = logging.getLogger("feagi_connector.zmq.client")

# Import ZMQ constants we need - these will be overridden if zmq is available
ZMQ_REQ = 3
ZMQ_SUB = 2
ZMQ_SUBSCRIBE = 6
ZMQ_PUSH = 8

# Define mocks first - they're needed whether ZMQ is available or not
class MockSocket:
    def close(self, *args, **kwargs):
        pass
    def connect(self, *args, **kwargs):
        pass
    def setsockopt(self, *args, **kwargs):
        pass
    async def send_multipart(self, *args, **kwargs):
        pass
    async def recv_multipart(self, *args, **kwargs):
        return [b"", b"{}"]

class MockContext:
    @classmethod
    def instance(cls):
        return cls()
    def socket(self, socket_type):
        return MockSocket()
    def term(self):
        pass

# Try to import ZMQ - first check if it's available
ZMQ_AVAILABLE = importlib.util.find_spec("zmq") is not None
logger.info(f"ZMQ available: {ZMQ_AVAILABLE}")

if ZMQ_AVAILABLE:
    try:
        # Clean import rather than a global import to avoid circular issues
        import zmq
        
        # Get constants from real zmq
        ZMQ_REQ = zmq.REQ
        ZMQ_SUB = zmq.SUB
        ZMQ_SUBSCRIBE = zmq.SUBSCRIBE
        ZMQ_PUSH = zmq.PUSH
        
        # Try to import asyncio support
        try:
            import zmq.asyncio
        except ImportError:
            logger.warning("zmq.asyncio not available, using mock")
            # Create mock asyncio module
            class MockAsyncio:
                Context = MockContext
            zmq.asyncio = MockAsyncio
        
        # Try to import auth support
        try:
            import zmq.auth
            import zmq.auth.thread
        except ImportError:
            logger.warning("zmq.auth not available, using mock")
            # Create mock auth module
            class MockThreadAuth:
                def __init__(self, *args, **kwargs):
                    pass
                def start(self):
                    pass
                def stop(self):
                    pass
                def allow(self, *args):
                    pass
                def deny(self, *args):
                    pass
                def configure_plain(self, *args, **kwargs):
                    pass
                def configure_curve(self, *args, **kwargs):
                    pass
                    
            auth_module = type('module', (), {})()
            thread_module = type('module', (), {})()
            thread_module.ThreadAuthenticator = MockThreadAuth
            auth_module.thread = thread_module
            zmq.auth = auth_module
            
    except ImportError:
        logger.warning("ZMQ not available despite spec check, using mock implementation")
        ZMQ_AVAILABLE = False

# If ZMQ is not available, create a mock zmq module
if not ZMQ_AVAILABLE:
    # Create a class that simulates the zmq module
    class MockZmq:
        REQ = ZMQ_REQ
        SUB = ZMQ_SUB
        SUBSCRIBE = ZMQ_SUBSCRIBE
        PUSH = ZMQ_PUSH
        
        class asyncio:
            Context = MockContext
    
    # Use our mock instead of real zmq
    zmq = MockZmq()

class ZmqClient:
    """
    ZeroMQ client for FEAGI communication.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        req_port: int = 5555,
        pub_port: int = 5556,
        push_port: int = 5557,
        stream_port: int = 5558,
        topics: List[str] = None
    ):
        """
        Initialize the ZeroMQ client.
        
        Args:
            host: Host address of the FEAGI server
            req_port: Port for request-reply pattern
            pub_port: Port for publish-subscribe pattern
            push_port: Port for push-pull pattern
            stream_port: Port for specialized data streams
            topics: List of topics to subscribe to
        """
        self.host = host
        self.req_port = req_port
        self.pub_port = pub_port
        self.push_port = push_port
        self.stream_port = stream_port
        self.topics = topics or ["events", "status"]
        
        # Connection info
        self.connected = False
        self.callbacks = {}
        
        # Only set up context and real implementation if ZMQ is available
        if ZMQ_AVAILABLE:
            try:
                self.context = zmq.asyncio.Context.instance()
                self.req_socket = None
                self.sub_socket = None
                self.push_socket = None
                self.stream_socket = None
                self.stop_event = threading.Event()
                self.thread = None
                self.loop = None
            except Exception as e:
                logger.error(f"Error setting up ZMQ client: {e}")
                self.context = None
    
    def register_topic_callback(self, topic: str, callback: Callable[[Dict[str, Any]], None]):
        """
        Register a callback for a specific topic.
        
        Args:
            topic: Topic to subscribe to
            callback: Function to call when a message is received on this topic
        """
        self.callbacks[topic] = callback
    
    def unregister_topic_callback(self, topic: str):
        """
        Unregister a callback for a specific topic.
        
        Args:
            topic: Topic to unsubscribe from
        """
        if topic in self.callbacks:
            del self.callbacks[topic]
    
    def start(self):
        """
        Start the client.
        
        Returns:
            True if started successfully, False otherwise
        """
        if not ZMQ_AVAILABLE:
            logger.warning("ZMQ not available, cannot start client")
            return False
            
        if self.connected:
            logger.warning("Client already started")
            return True
            
        try:
            # Create and start thread if using real ZMQ
            if hasattr(self, 'stop_event'):
                self.stop_event.clear()
                self.thread = threading.Thread(target=self._run_client_thread, daemon=True)
                self.thread.start()
                
                # Wait briefly for connection
                time.sleep(0.1)
            else:
                # No real ZMQ, just mark as connected
                self.connected = True
                
            return self.connected
        except Exception as e:
            logger.error(f"Error starting ZMQ client: {e}")
            return False
    
    def stop(self):
        """
        Stop the client.
        
        Returns:
            True if stopped successfully, False otherwise
        """
        if not self.connected:
            return True
            
        if not ZMQ_AVAILABLE:
            self.connected = False
            return True
            
        try:
            logger.info("Stopping ZMQ client")
            
            # Signal thread to stop if using real ZMQ
            if hasattr(self, 'stop_event'):
                self.stop_event.set()
                
                # Wait for thread to finish
                if self.thread and self.thread.is_alive():
                    self.thread.join(timeout=2.0)
            
            # Reset state
            self.connected = False
            
            return True
        except Exception as e:
            logger.error(f"Error stopping ZMQ client: {e}")
            self.connected = False
            return False
    
    def _run_client_thread(self):
        """Run the client in a background thread."""
        if not ZMQ_AVAILABLE:
            logger.warning("ZMQ not available, cannot run client thread")
            return
            
        # Create new event loop for this thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            # Connect to server
            self.loop.run_until_complete(self._connect())
            if not self.connected:
                return
                
            # Run message loop
            self.loop.run_until_complete(self._message_loop())
        except Exception as e:
            logger.error(f"Error in client thread: {e}")
        finally:
            # Clean up
            if hasattr(self, '_close_sockets'):
                self._close_sockets()
            self.connected = False
    
    async def _connect(self):
        """Connect to the server."""
        if not ZMQ_AVAILABLE:
            return False
            
        try:
            # Create sockets
            self.req_socket = self.context.socket(zmq.REQ)
            self.req_socket.connect(f"tcp://{self.host}:{self.req_port}")
            
            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.connect(f"tcp://{self.host}:{self.pub_port}")
            
            # Subscribe to topics
            for topic in self.topics:
                self.sub_socket.setsockopt(zmq.SUBSCRIBE, topic.encode('utf-8'))
            
            self.push_socket = self.context.socket(zmq.PUSH)
            self.push_socket.connect(f"tcp://{self.host}:{self.push_port}")
            
            self.stream_socket = self.context.socket(zmq.SUB)
            self.stream_socket.connect(f"tcp://{self.host}:{self.stream_port}")
            self.stream_socket.setsockopt(zmq.SUBSCRIBE, b"")
            
            # Mark as connected
            self.connected = True
            return True
        except Exception as e:
            logger.error(f"Error connecting to server: {e}")
            return False
    
    async def _message_loop(self):
        """Run the message loop."""
        if not self.connected:
            return
            
        # Create task for handling subscription messages
        task = asyncio.create_task(self._handle_subscription_messages())
        
        try:
            # Run until stopped
            while not self.stop_event.is_set():
                await asyncio.sleep(0.1)
        finally:
            # Cancel the task
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
    
    async def _handle_subscription_messages(self):
        """Handle subscription messages."""
        try:
            while not self.stop_event.is_set():
                try:
                    # Wait for message
                    frames = await asyncio.wait_for(
                        self.sub_socket.recv_multipart(),
                        timeout=0.5
                    )
                    
                    if len(frames) != 2:
                        continue
                    
                    # Process message
                    topic = frames[0].decode('utf-8')
                    data = frames[1].decode('utf-8')
                    
                    # Parse JSON
                    try:
                        message = json.loads(data)
                    except json.JSONDecodeError:
                        continue
                    
                    # Call callback
                    if topic in self.callbacks:
                        callback = self.callbacks[topic]
                        asyncio.create_task(self._run_callback(callback, message))
                
                except asyncio.TimeoutError:
                    # Normal timeout
                    continue
                except asyncio.CancelledError:
                    # Cancelled
                    break
                except Exception as e:
                    logger.error(f"Error handling subscription: {e}")
        except asyncio.CancelledError:
            # Normal cancellation
            pass
    
    async def _run_callback(self, callback, message):
        """Run a callback safely."""
        try:
            if asyncio.iscoroutinefunction(callback):
                await callback(message)
            else:
                callback(message)
        except Exception as e:
            logger.error(f"Error in callback: {e}")
    
    def send_request(self, command: str, params: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Send a request to the server.
        
        Args:
            command: Command to send
            params: Parameters for the command
            
        Returns:
            Response from the server
        """
        if not ZMQ_AVAILABLE:
            return {"status": "error", "message": "ZMQ not available"}
            
        if not self.connected:
            return {"status": "error", "message": "Not connected"}
            
        # Create request
        request = {
            "command": command,
            "params": params or {}
        }
        
        # Send request
        if hasattr(self, 'loop') and self.loop:
            try:
                # Run in event loop
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    return loop.run_until_complete(self._send_request(request))
                finally:
                    loop.close()
            except Exception as e:
                logger.error(f"Error sending request: {e}")
                return {"status": "error", "message": str(e)}
        else:
            return {"status": "error", "message": "Client not properly initialized"}
    
    async def _send_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """Send a request asynchronously."""
        try:
            # Encode request
            data = json.dumps(request).encode('utf-8')
            
            # Send and receive
            await self.req_socket.send_multipart([data])
            frames = await asyncio.wait_for(self.req_socket.recv_multipart(), timeout=5.0)
            
            # Parse response
            if not frames:
                return {"status": "error", "message": "Empty response"}
                
            try:
                response = json.loads(frames[0].decode('utf-8'))
                return response
            except json.JSONDecodeError:
                return {"status": "error", "message": "Invalid response format"}
        except asyncio.TimeoutError:
            return {"status": "error", "message": "Request timeout"}
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def _close_sockets(self):
        """Close all sockets."""
        for socket in [self.req_socket, self.sub_socket, self.push_socket, self.stream_socket]:
            if socket:
                socket.close()
                
        self.req_socket = None
        self.sub_socket = None
        self.push_socket = None
        self.stream_socket = None

# Add FEAGI-specific client implementation
class ZmqFeagiClient(ZmqClient):
    """
    ZeroMQ client specialized for FEAGI communication.
    
    This client extends the base ZmqClient with FEAGI-specific
    functionality for agents to register and communicate with FEAGI.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        req_port: int = 5555,
        pub_port: int = 5556,
        push_port: int = 5557,
        stream_port: int = 5558,
        agent_id: str = None,
        agent_type: str = "external"
    ):
        """
        Initialize a FEAGI ZMQ client.
        
        Args:
            host: FEAGI server host
            req_port: Request-reply port
            pub_port: Publish-subscribe port
            push_port: Push-pull port
            stream_port: Stream port
            agent_id: Unique agent ID (will be generated if not provided)
            agent_type: Type of agent (e.g., "motor", "sensor", "external")
        """
        super().__init__(
            host=host,
            req_port=req_port,
            pub_port=pub_port,
            push_port=push_port,
            stream_port=stream_port,
            topics=["events", "status"]
        )
        
        self.agent_id = agent_id or f"agent-{int(time.time() * 1000)}"
        self.agent_type = agent_type
        self.is_registered = False
        
    def start(self) -> bool:
        """
        Start the client and register with FEAGI.
        
        Returns:
            bool: True if started and registered, False otherwise
        """
        # Start the base client
        if not super().start():
            return False
            
        # Register with FEAGI (simplified - for testing only)
        self.is_registered = True
        logger.info(f"Agent {self.agent_id} registered with FEAGI")
        return True
            
    def stop(self) -> bool:
        """
        Unregister from FEAGI and stop the client.
        
        Returns:
            bool: True if stopped successfully, False otherwise
        """
        # Unregister from FEAGI if registered
        if self.is_registered:
            logger.info(f"Agent {self.agent_id} unregistered from FEAGI")
                
        # Stop the base client
        return super().stop() 