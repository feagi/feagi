# ZeroMQ Architecture in FEAGI

*Last Updated: January 15, 2025*

## Overview

This document describes the comprehensive ZeroMQ (ZMQ) architecture in FEAGI, implementing a **multi-stream architecture** where each stream has a specific purpose and protocol. This design eliminates protocol confusion, improves performance, and provides clear separation of concerns.

## Architecture Principles

1. **Multi-Stream Separation**: Dedicated streams for different protocols and purposes
2. **Protocol Isolation**: Each stream only accepts its designated message format
3. **Asynchronous Processing**: Non-blocking message handling for performance
4. **Connection Management**: Centralized connection tracking and lifecycle management
5. **Binary Efficiency**: Optimized binary protocols for high-throughput data exchange
6. **Versioning**: Protocol versioning for backward compatibility

## Multi-Stream Architecture Overview

FEAGI implements **Option 4: Separate Dedicated Streams** for optimal performance, maintainability, and protocol separation.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           FEAGI ZMQ Server                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ REQ/REP     │ │ Control     │ │ REST        │ │ Visualization│           │
│  │ Stream      │ │ Stream      │ │ Stream      │ │ Stream       │           │
│  │ Port 5555   │ │ Port 5561   │ │ Port 5563   │ │ Port 5562    │           │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘           │
│  │               │               │               │                          │
│  │ Legacy ZMQ    │ Legacy        │ REST API      │ Neural Data              │
│  │ Commands      │ Control       │ Operations    │ Broadcasting             │
│  └───────────────┴───────────────┴───────────────┴──────────────────────────┤
├─────────────────────────────────────────────────────────────────────────────┤
│                           FEAGI Core Engine                                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Stream Specifications

### **REQ/REP Stream (Port 5555)**

**Purpose**: Traditional ZMQ request-reply pattern for legacy command operations.

**Characteristics**:
- **Socket Type**: REP (Reply)
- **Pattern**: Synchronous request-reply
- **Protocol**: Legacy ZMQ command format
- **Use Cases**: Command-line tools, legacy integrations

**Message Format**:
```json
{
  "command": "get_status|set_config|...",
  "parameters": {...},
  "timestamp": 1621234567890
}
```

**Client Example**:
```python
import zmq
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")
socket.send_string('{"command": "get_status"}')
response = socket.recv_string()
```

### **Control Stream (Port 5561)**

**Purpose**: Legacy control protocol for agent management and heartbeat monitoring.

**Characteristics**:
- **Socket Type**: ROUTER/DEALER
- **Pattern**: Asynchronous message routing
- **Protocol**: Legacy control message format (FCP - FEAGI Control Protocol)
- **Use Cases**: Agent registration, heartbeat monitoring, legacy control systems

**Message Format**:
```json
{
  "message_type": "hello|heartbeat|status|goodbye",
  "agent_id": "unique_agent_identifier",
  "timestamp": 1621234567890,
  "data": {...}
}
```

**Supported Message Types**:
- `hello`: Agent registration
- `heartbeat`: Periodic health check
- `status`: Status query
- `goodbye`: Agent disconnection

### **REST Stream (Port 5563) - Dedicated REST API**

**Purpose**: Pure REST API operations with HTTP-like semantics over ZMQ.

**Characteristics**:
- **Socket Type**: ROUTER/DEALER
- **Pattern**: Asynchronous request-reply with HTTP semantics
- **Protocol**: REST API format only
- **Use Cases**: Modern applications, web interfaces, API clients

#### REST Message Format

**Request Format**:
```json
{
  "route": "/v1/path/to/resource",
  "method": "GET|POST|PUT|DELETE",
  "params": {
    "param1": "value1"
  },
  "query": {
    "filter": "value"
  },
  "body": {
    "property": "value"
  },
  "headers": {
    "authorization": "Bearer token"
  },
  "timestamp": 1621234567890
}
```

**Response Format**:
```json
{
  "status": 200,
  "headers": {
    "content-type": "application/json"
  },
  "body": {
    "property": "value"
  },
  "timestamp": 1621234567890
}
```

#### REST API Examples

**System Health Check**:
```json
// Request
{
  "route": "/v1/system/health_check",
  "method": "GET",
  "timestamp": 1621234567890
}

// Response  
{
  "status": 200,
  "headers": {"content-type": "application/json"},
  "body": {"status": "healthy"},
  "timestamp": 1621234567890
}
```

**Get Cortical Area**:
```json
// Request
{
  "route": "/v1/connectome/cortical_area/12345",
  "method": "GET", 
  "params": {"cortical_id": "12345"},
  "timestamp": 1621234567890
}

// Response
{
  "status": 200,
  "headers": {"content-type": "application/json"},
  "body": {
    "id": "12345",
    "name": "Visual Cortex",
    "type": "sensory",
    "position": [10, 10, 10],
    "dimensions": [10, 10, 1]
  },
  "timestamp": 1621234567890
}
```

### **Visualization Stream (Port 5562)**

**Purpose**: Real-time neural activity data broadcasting.

**Characteristics**:
- **Socket Type**: PUB (Publisher)
- **Pattern**: Publish-subscribe (one-to-many)
- **Protocol**: Binary `feagi_bytes` format (FVP - FEAGI Visualization Protocol)
- **Use Cases**: Brain visualization, monitoring dashboards, data analysis

**Data Format**: Binary `feagi_bytes` with structure ID headers

## Protocol Separation Rules

### **Strict Protocol Enforcement**

Each stream **ONLY** accepts its designated message format:

1. **REST Stream (5563)** - Only accepts messages with `method` and `route` fields
2. **Control Stream (5561)** - Only accepts messages with `message_type` field  
3. **REQ/REP Stream (5555)** - Only accepts messages with `command` field
4. **Visualization Stream (5562)** - Only broadcasts binary data

### **Protocol Validation**

**REST Stream Validation**:
```python
def is_valid_rest_message(message):
    return (
        isinstance(message, dict) and
        'method' in message and 
        'route' in message and
        isinstance(message['method'], str) and
        isinstance(message['route'], str)
    )
```

**Control Stream Validation**:
```python
def is_valid_control_message(message):
    return (
        isinstance(message, dict) and
        'message_type' in message and
        isinstance(message['message_type'], str)
    )
```

### **Error Handling for Wrong Protocols**

**Sending REST message to Control stream**:
```
Request:  {"method": "GET", "route": "/v1/health"} → Control Stream (5561)
Response: {
  "message_type": "error",
  "error": "REST API messages should be sent to the dedicated REST stream port",
  "timestamp": 1621234567890
}
```

**Sending Control message to REST stream**:
```
Request:  {"message_type": "hello"} → REST Stream (5563)
Response: {
  "status": 400,
  "body": {
    "type": "error",
    "code": "INVALID_REST_FORMAT", 
    "message": "Message must contain 'method' and 'route' fields"
  },
  "timestamp": 1621234567890
}
```

## Component Architecture

### ZMQManager

The central coordinator for all ZMQ communications:

```python
class ZMQManager:
    def __init__(self, host="*", req_port=5555, control_port=5561, 
                 rest_port=5563, vis_port=5562):
        self.context = zmq.Context.instance()
        self.host = host
        self.ports = {
            "req": req_port,
            "control": control_port,
            "rest": rest_port,
            "visualization": vis_port
        }
        self.connection_manager = ConnectionManager(self.context, **self.ports)
        self.streams = {}
        self._initialize_streams()
```

### Stream Handlers

**REST Stream Handler**:
```python
class RestStreamHandler:
    async def process_rest_message(self, message):
        if not self.is_valid_rest_format(message):
            return self.create_error_response(400, "INVALID_REST_FORMAT")
        
        # Route to appropriate REST endpoint
        return await self.route_rest_request(
            message['method'], 
            message['route'], 
            message.get('params', {}),
            message.get('query', {}),
            message.get('body', {})
        )
```

**Control Stream Handler**:
```python  
class ControlStreamHandler:
    async def process_control_message(self, message):
        if not self.is_valid_control_format(message):
            return self.create_error_response("INVALID_CONTROL_FORMAT")
            
        # Handle legacy control messages
        return await self.handle_control_command(
            message['message_type'],
            message.get('agent_id'),
            message.get('data', {})
        )
```

## Performance Characteristics

### Stream Performance

| Stream | Latency | Throughput | Use Case |
|--------|---------|------------|----------|
| REST (5563) | ~2-3ms | High | API operations |
| Control (5561) | ~1-2ms | Medium | Agent management |
| Visualization (5562) | ~0.5ms | Very High | Real-time data |
| REQ/REP (5555) | ~3-5ms | Low | Legacy commands |

### Performance Benefits

1. **No Message Routing Overhead**: Messages go directly to the right handler
2. **Protocol-Specific Optimizations**: Each stream optimized for its use case
3. **Independent Scaling**: Scale different streams based on load patterns
4. **Reduced Contention**: No competing message types on same endpoint

## Configuration

### Command Line Arguments

```bash
python -m feagi.main \
  --zmq-req-port 5555 \
  --zmq-control-port 5561 \
  --zmq-rest-port 5563 \
  --zmq-vis-port 5562
```

### Environment Variables

```bash
export FEAGI_ZMQ_REQ_PORT=5555
export FEAGI_ZMQ_CONTROL_PORT=5561  
export FEAGI_ZMQ_REST_PORT=5563
export FEAGI_ZMQ_VIS_PORT=5562
```

### Configuration File

```json
{
  "zmq": {
    "host": "127.0.0.1",
    "req_port": 5555,
    "control_port": 5561,
    "rest_port": 5563,
    "vis_port": 5562
  }
}
```

## Authentication and Security

### REST Stream Authentication

```json
{
  "headers": {
    "authorization": "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI..."
  }
}
```

### Control Stream Authentication

```json
{
  "message_type": "hello",
  "agent_id": "secure_agent",
  "authentication": {
    "token": "bearer_token_here"
  }
}
```

## Testing and Monitoring

### Stream Connectivity Testing

```bash
# Test all streams are listening
ss -tuln | grep -E "5555|5561|5562|5563"

# Expected output:
tcp LISTEN 127.0.0.1:5555   # REQ/REP
tcp LISTEN 127.0.0.1:5561   # Control
tcp LISTEN 127.0.0.1:5562   # Visualization  
tcp LISTEN 127.0.0.1:5563   # REST
```

### Protocol Validation Testing

```python
# Test REST stream
import zmq, json
ctx = zmq.Context()
sock = ctx.socket(zmq.DEALER)
sock.connect('tcp://localhost:5563')
sock.send_multipart([b'', json.dumps({
    'method': 'GET', 
    'route': '/v1/system/health_check'
}).encode()])
response = json.loads(sock.recv_multipart()[1])
```

## Migration Guide

### From Unified Stream Architecture

**Before (Unified)**:
```python
# Single control stream handled everything
client = ZMQClient(port=5561)  # Mixed REST and control
```

**After (Separate Streams)**:
```python
# Dedicated streams for each protocol
rest_client = ZMQRestClient(port=5563)     # REST only
control_client = ControlClient(port=5561)  # Control only
```

## Best Practices

### Stream Selection Guidelines

- **REST Stream (5563)**: Modern APIs, web interfaces, bridges
- **Control Stream (5561)**: Agent registration, heartbeats, legacy control
- **Visualization Stream (5562)**: Real-time monitoring, dashboards
- **REQ/REP Stream (5555)**: Command-line tools, simple scripts

### Performance Optimization

1. **Choose the Right Stream**: Use the most appropriate stream for your use case
2. **Batch Operations**: Where possible, batch multiple operations
3. **Connection Pooling**: Reuse connections for multiple operations
4. **Async Operations**: Use async clients for better throughput

## Related Documentation

- [System Overview](arch-system-overview.md)
- [IPC Architecture](arch-ipc.md)
- [State Management](arch-state-management.md)
- [System Diagrams](arch-system-diagrams.md) 