# ZMQ Streams for FEAGI 2.0

## Overview

This package contains production-ready ZMQ stream implementations for FEAGI 2.0, designed with clean architecture principles, thread safety, and high performance.

## Key Features

### 🚀 Production Ready
- **Thread-safe operation** with proper race condition protection
- **Robust error handling** and graceful shutdown
- **Performance monitoring** and health status reporting
- **Clean logging** suitable for production environments

### 🏗️ Clean Architecture
- **Integration with existing ZMQ patterns** from `feagi.api.zmq.patterns`
- **CoreAPIService integration** for FEAGI access
- **Proper base class hierarchy** for extending streams
- **No code duplication** - unified implementation

### ⚡ High Performance
- **Optimized threading** with responsive shutdown (< 2 seconds)
- **Efficient socket management** using existing ZMQ infrastructure
- **Race condition protection** preventing crashes during shutdown
- **Memory efficient** with proper resource cleanup

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           FEAGI ZMQ Streams                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────┐ │
│  │ Sensory     │ │ Motor       │ │ Visualization│ │ Control     │ │ REST    │ │
│  │ Stream      │ │ Stream      │ │ Stream      │ │ Stream      │ │ Stream  │ │
│  │ Port 5558   │ │ Port 9050   │ │ Port 5562   │ │ Port 5561   │ │Port 5563│ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ └─────────┘ │
│  │               │               │               │               │           │ │
│  │ PULL          │ PUB           │ PUB           │ ROUTER/DEALER │ ROUTER/  │ │
│  │ Sensor Data   │ Motor Cmds    │ Brain Activity│ Agent Control │ DEALER   │ │
│  │               │               │               │               │ REST API │ │
│  └───────────────┴───────────────┴───────────────┴───────────────┴─────────┘ │
├─────────────────────────────────────────────────────────────────────────────┤
│                    Existing ZMQ Patterns Integration                         │
│  ┌─────────────────────────────┐   ┌─────────────────────────────────────┐  │
│  │   feagi.api.zmq.patterns    │   │        CoreAPIService               │  │
│  │ • PublisherServer           │   │ • Direct FEAGI integration          │  │
│  │ • SubscriberClient          │   │ • No abstraction layer             │  │
│  │ • PushServer/PullClient     │   │ • Production tested               │  │
│  │ • RequestReplyManager       │   │ • Existing error handling         │  │
│  └─────────────────────────────┘   └─────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────────────────┤
│                           FEAGI Core Engine                                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Stream Types

### 1. Sensory Stream (Inbound)

Handles incoming sensor data from external sources.

**Features:**
- High-throughput data ingestion
- Multiple sensor type support
- Data validation and preprocessing
- Thread-safe operation

**Usage:**
```python
from feagi.api.zmq.streams import SensoryStream

stream = SensoryStream(
    host="*",
    port=5558,
    core_api=core_api_instance
)
stream.start()
```

### 2. Motor Stream (Outbound)

Sends motor commands to external controllers.

**Features:**
- Real-time motor command dispatch
- Safety validation and limits
- Multiple command formats
- High-frequency operation

**Usage:**
```python
from feagi.api.zmq.streams import MotorStream

stream = MotorStream(
    host="*",
    port=9050,
    core_api=core_api_instance
)
stream.start()
```

### 3. Visualization Stream (Outbound)

Broadcasts brain state data for visualization.

**Features:**
- High-frequency brain state updates
- Multiple data formats
- Client heartbeat management
- Automatic FQ sampler control
- Race condition protection

**Usage:**
```python
from feagi.api.zmq.streams import VisualizationStream

stream = VisualizationStream(
    host="*",
    port=5562,
    core_api=core_api_instance,
    fq_sampler=fq_sampler_instance
)
stream.start()
```

### 4. Control Stream (Bidirectional)

Handles control requests and responses.

**Features:**
- Agent registration and management
- Request/response correlation
- Session management
- Heartbeat monitoring

**Usage:**
```python
from feagi.api.zmq.streams import ControlStream

stream = ControlStream(
    host="*",
    port=5561,
    core_api=core_api_instance
)
stream.start()
```

### 5. REST Stream (Bidirectional)

Handles HTTP-like REST API operations.

**Features:**
- REST API semantics
- HTTP status codes
- JSON request/response
- Scalable operation

**Usage:**
```python
from feagi.api.zmq.streams import RestStream

stream = RestStream(
    host="*",
    port=5563,
    core_api=core_api_instance
)
stream.start()
```

## Factory Functions

### Stream Manager

Use the stream manager for complete setup:

```python
from feagi.api.zmq.streams import initialize_all_streams, shutdown_all_streams

# Configuration for all streams
config = {
    'sensory': {'port': 5558},
    'motor': {'port': 9050},
    'visualization': {'port': 5562},
    'control': {'port': 5561},
    'rest': {'port': 5563}
}

# Initialize all streams
streams = initialize_all_streams(
    core_api=core_api_instance,
    host="*",
    stream_configs=config
)

try:
    # Run your application
    time.sleep(60)
finally:
    # Clean shutdown
    shutdown_all_streams(streams)
```

### Individual Stream Creation

```python
from feagi.api.zmq.streams import create_stream_manager

# Create specific streams
streams = create_stream_manager(
    core_api=core_api_instance,
    host="*",
    stream_configs={
        'visualization': {'port': 5562},
        'control': {'port': 5561}
    }
)

# Start individually
for stream in streams.values():
    stream.start()
```

## Base Classes

### UnidirectionalStream

For streams with one-way data flow (sensory, motor, visualization):

```python
from feagi.api.zmq.streams.base_stream import UnidirectionalStream, SocketType, DataDirection

class MyStream(UnidirectionalStream):
    def __init__(self, host="*", port=5559, core_api=None):
        super().__init__(
            host=host,
            port=port,
            direction=DataDirection.OUTBOUND,
            socket_type=SocketType.PUB,
            core_api=core_api
        )
    
    def _data_worker(self):
        # Implement your data processing
        pass
```

### BidirectionalStream

For streams with request/response patterns (control, REST):

```python
from feagi.api.zmq.streams.base_stream import BidirectionalStream

class MyStream(BidirectionalStream):
    def __init__(self, host="*", port=5559, core_api=None):
        super().__init__(
            host=host,
            port=port,
            core_api=core_api
        )
    
    async def _handle_request(self, client_id, message):
        # Implement request handling
        return {"response": "data"}
```

## Performance Monitoring

### Stream Statistics

All streams provide detailed statistics:

```python
stats = stream.get_stats()
print(f"Messages processed: {stats['messages_processed']}")
print(f"Bytes processed: {stats['bytes_processed']}")
print(f"Error count: {stats['errors']}")
print(f"Uptime: {stats['uptime_seconds']}")
```

### Health Status

Get comprehensive health information:

```python
# For visualization stream (has detailed health status)
health = stream.get_health_status()
print(f"Running: {health['running']}")
print(f"Socket available: {health['socket_available']}")
print(f"Worker threads: {health['worker_thread_count']}")
print(f"Queue size: {health.get('queue_size', 'N/A')}")
```

### Real-time Monitoring

```python
def monitor_streams(streams, interval=5.0):
    """Monitor stream performance in real-time."""
    while True:
        for name, stream in streams.items():
            if hasattr(stream, 'get_stats'):
                stats = stream.get_stats()
                print(f"Stream {name}:")
                print(f"  Messages: {stats.get('messages_processed', 0)}")
                print(f"  Errors: {stats.get('errors', 0)}")
                print(f"  Uptime: {stats.get('uptime_seconds', 0):.1f}s")
        
        time.sleep(interval)
```

## Error Handling

### Graceful Shutdown

All streams support graceful shutdown:

```python
def shutdown_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    logger.info("Received shutdown signal")
    shutdown_all_streams(streams)
    sys.exit(0)

import signal
signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)
```

### Error Recovery

```python
def robust_stream_operation(stream):
    """Example of robust stream operation with error handling."""
    max_retries = 3
    retry_delay = 1.0
    
    for attempt in range(max_retries):
        try:
            stream.start()
            return True
        except Exception as e:
            logger.error(f"Stream start failed (attempt {attempt + 1}): {e}")
            if attempt == max_retries - 1:
                return False
            time.sleep(retry_delay * (attempt + 1))
    
    return False
```

## Testing

### Unit Tests

Run the comprehensive test suite:

```bash
cd feagi_core
pytest tests/unit/test_zmq_streams.py -v
```

### Integration Testing

```python
def test_stream_integration():
    """Test stream integration with CoreAPIService."""
    from feagi.api.core.services.core_api_service import CoreAPIService
    
    core_api = CoreAPIService()
    stream = VisualizationStream(
        host="*",
        port=5562,
        core_api=core_api
    )
    
    try:
        stream.start()
        assert stream.running
        
        # Test functionality
        health = stream.get_health_status()
        assert health['running']
        assert health['socket_available']
        
    finally:
        stream.stop()
```

## Migration Guide

### From Previous Versions

The architecture has been unified - no more duplicate files:

```python
# OLD - Multiple conflicting implementations
from feagi.api.zmq.streams.visualization_refactored import VisualizationStream

# NEW - Single unified implementation  
from feagi.api.zmq.streams import VisualizationStream
```

### Key Changes

1. **Unified Implementation**: No more `*_refactored.py` or `*_v2.py` files
2. **CoreAPIService Integration**: Direct integration, no abstraction layer
3. **Existing Patterns**: Uses proven `feagi.api.zmq.patterns` infrastructure
4. **Thread Safety**: Built-in race condition protection
5. **Production Ready**: Clean logging and proper error handling

## Best Practices

### 1. Use Factory Functions

```python
# ✅ Recommended
streams = initialize_all_streams(core_api=core_api)

# ❌ Not recommended (manual setup)
stream = VisualizationStream(host="*", port=5562, ...)
stream.start()
```

### 2. Proper Lifecycle Management

```python
def run_stream_application():
    streams = initialize_all_streams(core_api=core_api)
    
    try:
        # Run application logic
        while True:
            # Your application code
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("Shutting down gracefully...")
    finally:
        shutdown_all_streams(streams)
```

### 3. Monitor Performance

```python
# Regular health checks
for name, stream in streams.items():
    if hasattr(stream, 'get_health_status'):
        health = stream.get_health_status()
        if not health['running']:
            logger.warning(f"Stream {name} not running")
```

## Troubleshooting

### Common Issues

#### 1. Stream Won't Start

```python
try:
    stream.start()
except Exception as e:
    logger.error(f"Stream start failed: {e}")
    # Check port availability, permissions, etc.
```

#### 2. High Error Rates

```python
stats = stream.get_stats()
error_rate = stats.get('errors', 0) / max(stats.get('messages_processed', 1), 1)
if error_rate > 0.1:  # >10% error rate
    logger.warning(f"High error rate: {error_rate:.2%}")
```

#### 3. Shutdown Hangs

The new architecture prevents shutdown hangs with responsive threading:

```python
# Automatic timeout handling
stream.stop()  # Should complete in < 2 seconds
```

### Debugging

Enable detailed logging:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Streams will log detailed operation information
```

Monitor thread state:

```python
health = stream.get_health_status()
for thread_info in health.get('worker_threads', []):
    print(f"Thread {thread_info['name']}: alive={thread_info['alive']}")
```

## Contributing

When extending the stream architecture:

1. **Use Existing Patterns**: Leverage `feagi.api.zmq.patterns`
2. **Follow Base Classes**: Extend `UnidirectionalStream` or `BidirectionalStream`
3. **Thread Safety**: Use proper locking and race condition protection
4. **Add Tests**: Include comprehensive unit and integration tests
5. **Update Documentation**: Keep this README current

## License

Copyright 2025 Neuraville Inc. Licensed under Apache 2.0. 