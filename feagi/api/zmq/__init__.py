"""
ZeroMQ Interface for FEAGI API

This package provides ZeroMQ-based interfaces for high-performance
communication with FEAGI, including:

- Request-Reply pattern for CRUD operations
- Publish-Subscribe pattern for events and updates
- Push-Pull pattern for high-throughput data processing
- Specialized streams for sensorimotor data and visualization
"""

import os
from feagi.utils.logger import setup_logger
logger = setup_logger()
import asyncio
import threading
from typing import Optional, List, Any

from .server import ZmqServer
from .client import ZmqClient

from .patterns import (
    RequestReplyServer, RequestReplyClient, RequestReplyManager,
    PublisherServer, SubscriberClient, PubSubManager,
    PushServer, PullClient, PushPullManager
)

from .streams import (
    SensorimotorStream, SensorimotorClient,
    VisualizationStream, VisualizationClient
)



def create_zmq_server(
    host: str = None,
    pub_port: int = None, 
    sub_port: int = None,
    topics=None,
    use_auth=False,
    use_encryption=False,
    config=None
):
    """
    Create and initialize a ZMQ server instance.
    
    This is the main factory function for creating ZMQ servers in FEAGI.
    
    Args:
        host: Host address to bind to
        pub_port: Port for the publisher socket
        sub_port: Port for the subscriber socket  
        topics: List of topics to support
        use_auth: Whether to use authentication (deprecated)
        use_encryption: Whether to use encryption (deprecated)
        config: Additional configuration (deprecated)
        
    Returns:
        Initialized ZmqServer instance or None if initialization fails
    """
    # Log deprecation warnings for removed parameters
    if use_auth:
        logger.warning("The 'use_auth' parameter is deprecated and will be ignored.")
    if use_encryption:
        logger.warning("The 'use_encryption' parameter is deprecated and will be ignored.")
    if config:
        logger.warning("The 'config' parameter is deprecated and will be ignored.")
    
    try:
        # Import the CoreAPIService stub if needed
        try:
            from ..core.service import CoreApiService
        except ImportError:
            # Create a stub class 
            class CoreApiService:
                def __init__(self):
                    pass
                
                async def get_simulation_status(self):
                    return {"running": False, "burst_count": 0}
                    
                async def get_performance_stats(self):
                    return {"cpu": 0, "memory": 0}
                    
                async def get_system_events(self):
                    return {"events": []}
                    
                async def get_log_events(self):
                    return {"logs": []}
                    
                async def get_brain_activity(self):
                    return {}
                    
                async def get_brain_structure(self):
                    return {}
                
                async def get_system_metrics(self):
                    return {"cpu": 0, "memory": 0, "gpu": 0}
        
        # Create core API service
        core_api = CoreApiService()
        
        # Create the server
        server = ZmqServer(
            core_api=core_api,
            host=host or "127.0.0.1",
            req_rep_port=5555,  # Default req_rep port
            pub_sub_port=pub_port or 5556,
            push_pull_port=sub_port or 5557,  # Reuse sub_port for push_pull for compatibility
            sensorimotor_port=5558,
            vis_base_port=5560
        )
        
        # Create a wrapper class to mimic the old ZMQServer API
        class ZmqServerWrapper:
            def __init__(self, server):
                self.server = server
                self.running = False
            
            def start(self):
                """Start the server"""
                # Create and run a background thread to run the server
                def run_server():
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    loop.run_until_complete(self.server.start())
                    loop.run_forever()
                
                self.thread = threading.Thread(target=run_server, daemon=True)
                self.thread.start()
                self.running = True
                return True
            
            def shutdown(self):
                """Shutdown the server"""
                if self.running:
                    # Create a new event loop for shutdown
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    loop.run_until_complete(self.server.stop())
                    loop.close()
                    self.running = False
            
            def publish(self, topic, message):
                """Publish a message to a topic"""
                # Create a new event loop for publishing
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.server.pub_sub.publish_event(topic, message))
                loop.close()
                return True
            
            def subscribe(self, topic, callback):
                """Subscribe to a topic (stub - not supported in wrapper)"""
                logger.warning("subscribe() not supported in ZmqServer wrapper")
                return False
            
            def is_healthy(self):
                """Check if the server is healthy"""
                return self.running
        
        # Wrap the server
        return ZmqServerWrapper(server)
        
    except Exception as e:
        logger.error(f"Failed to create ZMQServer: {e}")
        
    return None


def create_zmq_client(
    host: str = None,
    pub_port: int = None,
    sub_port: int = None,
    topics: List[str] = None
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
        client = ZmqClient(
            host=host or os.environ.get("FEAGI_ZMQ_HOST", "127.0.0.1"),
            req_port=5555,  # Default req_port
            pub_port=pub_port or int(os.environ.get("FEAGI_ZMQ_PUB_PORT", "5556")),
            push_port=sub_port or int(os.environ.get("FEAGI_ZMQ_SUB_PORT", "5557")),
            stream_port=5558,
            topics=topics
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
                response = self.client.send_request("publish", {
                    "topic": topic,
                    "message": message
                })
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

__all__ = [
    # Factory functions
    'create_zmq_server',
    'create_zmq_client',
    
    # Main server/client
    'ZmqServer',
    'ZmqClient',
    
    # Request-Reply Pattern
    'RequestReplyServer',
    'RequestReplyClient',
    'RequestReplyManager',
    
    # Publish-Subscribe Pattern
    'PublisherServer',
    'SubscriberClient',
    'PubSubManager',
    
    # Push-Pull Pattern
    'PushServer',
    'PullClient',
    'PushPullManager',
    
    # Specialized Streams
    'SensorimotorStream',
    'SensorimotorClient',
    'VisualizationStream',
    'VisualizationClient',
] 