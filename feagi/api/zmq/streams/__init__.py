"""
ZeroMQ Stream Implementations for FEAGI API

This package contains stream implementations for different ZMQ communication patterns:
- Sensory streams for receiving sensor data
- Motor streams for sending motor commands
- Visualization streams for broadcasting brain activity
- Control streams for bidirectional agent communication
- REST streams for HTTP-like API operations

Architecture:
- Streams use existing ZMQ patterns from feagi.api.zmq.patterns
- Integration with CoreAPIService for FEAGI access
- Thread-safe operation with proper race condition protection
- Production-ready error handling and monitoring
"""

# Import base classes for extending streams
from .base_stream import (
    BaseZMQStream,
    BidirectionalStream,
    DataDirection,
    SocketType,
    StreamMode,
    UnidirectionalStream,
)
from .motor import MotorStream
from .rest import RestStream

# Import actual streams from their files
from .sensory_neural import SensoryNeuralStream as SensoryStream
from .visualization import VisualizationStream

__all__ = [
    # Main stream implementations
    "SensoryStream",
    "MotorStream",
    "VisualizationStream",
    "RestStream",
    # Base classes for extending
    "BaseZMQStream",
    "UnidirectionalStream",
    "BidirectionalStream",
    # Types and configuration
    "StreamMode",
    "SocketType",
    "DataDirection",
    # Factory functions
    "create_stream_manager",
    "initialize_all_streams",
    "shutdown_all_streams",
]


def create_stream_manager(core_api=None, host="*", stream_configs=None):
    """
    Create a complete stream manager with all FEAGI streams.

    Args:
        core_api: CoreAPIService instance for FEAGI access
        host: Host to bind streams to
        stream_configs: Dictionary of stream configurations

    Returns:
        Dictionary of initialized streams
    """
    if stream_configs is None:
        stream_configs = {
            "sensory": {"port": 5558},
            "motor": {"port": 9050},
            "visualization": {"port": 5562},
            "rest": {"port": 5563},
        }

    streams = {}

    # Create sensory stream
    if "sensory" in stream_configs:
        config = stream_configs["sensory"]
        streams["sensory"] = SensoryStream(
            host=host, port=config.get("port", 5558), core_api=core_api
        )

    # Create motor stream
    if "motor" in stream_configs:
        config = stream_configs["motor"]
        streams["motor"] = MotorStream(
            host=host, port=config.get("port", 9050), core_api=core_api
        )

    # Create visualization stream
    if "visualization" in stream_configs:
        config = stream_configs["visualization"]
        streams["visualization"] = VisualizationStream(
            host=host, port=config.get("port", 5562), core_api=core_api
        )

    # Create REST stream
    if "rest" in stream_configs:
        config = stream_configs["rest"]
        streams["rest"] = RestStream(
            host=host, port=config.get("port", 5563), core_api=core_api
        )

    return streams


def initialize_all_streams(core_api=None, host="*", stream_configs=None):
    """
    Initialize and start all FEAGI streams.

    Args:
        core_api: CoreAPIService instance for FEAGI access
        host: Host to bind streams to
        stream_configs: Dictionary of stream configurations

    Returns:
        Dictionary of running streams
    """
    streams = create_stream_manager(core_api, host, stream_configs)

    # Start all streams
    for name, stream in streams.items():
        try:
            stream.start()
        except Exception as e:
            raise RuntimeError(f"Failed to start {name} stream: {e}") from e

    return streams


def shutdown_all_streams(streams):
    """
    Shutdown all streams gracefully.

    Args:
        streams: Dictionary of stream instances
    """
    for name, stream in streams.items():
        try:
            if hasattr(stream, "stop"):
                stream.stop()
            elif hasattr(stream, "shutdown"):
                stream.shutdown()
        except Exception as e:
            print(f"Error shutting down {name} stream: {e}")


# Version information
__version__ = "2.0.0"
__architecture__ = "Production ZMQ Streams"
