# FEAGI ZMQ Server

This directory contains the ZeroMQ (ZMQ) server implementation for FEAGI. The server provides high-performance, real-time communication channels between FEAGI and external clients.

## Key Components

- `server.py`: Main ZMQ server implementation with proper event loop and thread management
- `client.py`: Client implementation for connecting to the ZMQ server
- `patterns/`: ZMQ communication patterns (req-rep, pub-sub, push-pull)
- `streams/`: Specialized streams for sensorimotor data and visualization

## Implementation Details

### Proper Event Loop Management

The ZMQ server implementation carefully manages asyncio event loops to avoid common issues like:

- "Task got Future attached to a different loop"
- "ZmqServer object has no attribute 'shutdown'"
- Thread safety problems with asyncio resources

The server creates a dedicated thread with its own event loop to handle all ZMQ communications, ensuring that:

1. Each thread has its own independent event loop
2. Asyncio objects are never shared across threads
3. Proper shutdown and cleanup are implemented
4. Coroutines are always run in the correct event loop

### Communication Patterns

The server implements multiple ZMQ communication patterns:

- **Request-Reply**: For traditional RPC-style operations
- **Publish-Subscribe**: For broadcasting events and updates
- **Push-Pull**: For high-throughput data processing

### Usage

The ZMQ server is typically started by the FEAGI process manager and provides several methods:

```python
# Start the server
zmq_server.start()

# Publish an event
await zmq_server.publish_event("event.type", {"data": "value"})

# Queue work
await zmq_server.queue_work("work.type", data, priority=5)

# Shutdown the server
zmq_server.shutdown()
```

## Thread Safety

The server implementation is thread-safe and can be used from any thread in the application. Methods that require asyncio operations are properly scheduled in the server's event loop using `asyncio.run_coroutine_threadsafe()`.

## Error Handling

Comprehensive error handling and logging are implemented throughout the server to ensure robustness and make it easier to diagnose issues. 