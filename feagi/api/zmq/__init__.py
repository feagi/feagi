"""
ZeroMQ Interface for FEAGI API

This package provides ZeroMQ-based interfaces for high-performance
communication with FEAGI, including:

- Request-Reply pattern for CRUD operations
- Publish-Subscribe pattern for events and updates
- Push-Pull pattern for high-throughput data processing
- Specialized streams for sensorimotor data and visualization
"""

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

__all__ = [
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