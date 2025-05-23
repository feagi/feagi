# FEAGI ZMQ Streams Architecture

This document provides a comprehensive overview of FEAGI's ZeroMQ (ZMQ) streams architecture, implementing **Option 4: Separate Dedicated Streams** for optimal performance, maintainability, and protocol separation.

## 🏗️ Architecture Overview

FEAGI implements a **multi-stream ZMQ architecture** where each stream has a specific, well-defined purpose and protocol. This design eliminates protocol confusion, improves performance, and provides clear separation of concerns.

### 📊 Complete Stream Architecture

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

## 🎯 Stream Responsibilities

### **⚡ REQ/REP Stream (Port 5555)**

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

### **🔧 Control Stream (Port 5561)**

**Purpose**: Legacy control protocol for agent management and heartbeat monitoring.

**Characteristics**:
- **Socket Type**: ROUTER/DEALER
- **Pattern**: Asynchronous message routing
- **Protocol**: Legacy control message format
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

**Client Example**:
```python
import zmq
import json
context = zmq.Context()
socket = context.socket(zmq.DEALER)
socket.setsockopt(zmq.IDENTITY, b"my_agent")
socket.connect("tcp://localhost:5561")

# Send registration
message = {
    "message_type": "hello",
    "agent_id": "my_agent",
    "agent_type": "godot_bridge"
}
socket.send_multipart([b"", json.dumps(message).encode()])
response = socket.recv_multipart()
```

### **🌐 REST Stream (Port 5563) - NEW DEDICATED STREAM**

**Purpose**: Pure REST API operations with HTTP-like semantics over ZMQ.

**Characteristics**:
- **Socket Type**: ROUTER/DEALER
- **Pattern**: Asynchronous request-reply with HTTP semantics
- **Protocol**: REST API format only
- **Use Cases**: Modern applications, web interfaces, API clients

**Message Format**:
```json
{
  "method": "GET|POST|PUT|DELETE",
  "route": "/v1/path/to/resource",
  "timestamp": 1621234567890,
  "params": {...},          // Path parameters
  "query": {...},           // Query parameters  
  "body": {...},            // Request body
  "headers": {...}          // Request headers
}
```

**Response Format**:
```json
{
  "status": 200,
  "headers": {
    "content-type": "application/json"
  },
  "body": {...},
  "timestamp": 1621234567890
}
```

**Supported Endpoints**:
- `GET /v1/system/health_check`: System health status
- `GET /v1/system/versions`: Version information
- `GET /v1/genome/file_name`: Current genome file
- `GET /v1/connectome/cortical_areas`: All cortical areas
- `GET /v1/morphology/morphology_list`: Available morphologies

**Client Example**:
```python
import zmq
import json
context = zmq.Context()
socket = context.socket(zmq.DEALER)
socket.connect("tcp://localhost:5563")

# Send REST API request
request = {
    "method": "GET",
    "route": "/v1/system/health_check",
    "timestamp": int(time.time() * 1000)
}
socket.send_multipart([b"", json.dumps(request).encode()])
response_parts = socket.recv_multipart()
response = json.loads(response_parts[1])
```

### **📺 Visualization Stream (Port 5562)**

**Purpose**: Real-time neural activity data broadcasting.

**Characteristics**:
- **Socket Type**: PUB (Publisher)
- **Pattern**: Publish-subscribe (one-to-many)
- **Protocol**: Binary `feagi_bytes` format
- **Use Cases**: Brain visualization, monitoring dashboards, data analysis

**Data Format**: Binary `feagi_bytes` with structure ID headers

**Client Example**:
```python
import zmq
from feagi_bytes import ByteStructureDecoder

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages
socket.connect("tcp://localhost:5562")

decoder = ByteStructureDecoder()
while True:
    data = socket.recv()
    decoded = decoder.decode_neuron_flat(data)
    # Process neural activity data
```

## 🔄 Protocol Separation Rules

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

## 🚀 Performance Benefits

### **Dedicated Processing Pipelines**

Each stream has its own processing pipeline optimized for its protocol:

1. **No Message Routing Overhead**: Messages go directly to the right handler
2. **Protocol-Specific Optimizations**: Each stream optimized for its use case
3. **Independent Scaling**: Scale different streams based on load patterns
4. **Reduced Contention**: No competing message types on same endpoint

### **Performance Characteristics**

| Stream | Latency | Throughput | Use Case |
|--------|---------|------------|----------|
| REST (5563) | ~2-3ms | High | API operations |
| Control (5561) | ~1-2ms | Medium | Agent management |
| Visualization (5562) | ~0.5ms | Very High | Real-time data |
| REQ/REP (5555) | ~3-5ms | Low | Legacy commands |

## 🛠️ Implementation Details

### **Stream Initialization Order**

1. **REQ/REP Stream** - Started first for immediate command access
2. **Control Stream** - Started second for agent registration
3. **REST Stream** - Started third for API operations
4. **Visualization Stream** - Started last, depends on core being ready

### **Error Recovery**

Each stream implements independent error recovery:

```python
class RestStream:
    async def _process_rest_messages(self):
        while self.running:
            try:
                # Process message
                await self._handle_rest_message()
            except Exception as e:
                logger.error(f"REST stream error: {e}")
                # Stream-specific error handling
                await asyncio.sleep(0.1)  # Brief backoff
```

### **Statistics Tracking**

Each stream maintains independent statistics:

```python
# REST Stream Statistics
{
    'requests_processed': 245,
    'requests_success': 243,
    'requests_error': 2,
    'success_rate': 99.2,
    'requests_per_second': 24.5
}

# Control Stream Statistics  
{
    'messages_processed': 45,
    'agents_registered': 3,
    'heartbeats_received': 42,
    'uptime_seconds': 3600
}
```

## 📋 Configuration

### **Command Line Arguments**

```bash
python -m feagi.main \
  --zmq-req-port 5555 \
  --zmq-control-port 5561 \
  --zmq-rest-port 5563 \
  --zmq-vis-port 5562
```

### **Environment Variables**

```bash
export FEAGI_ZMQ_REQ_PORT=5555
export FEAGI_ZMQ_CONTROL_PORT=5561  
export FEAGI_ZMQ_REST_PORT=5563
export FEAGI_ZMQ_VIS_PORT=5562
```

### **Configuration File**

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

## 🧪 Testing and Validation

### **Stream Connectivity Testing**

```bash
# Test all streams are listening
ss -tuln | grep -E "5555|5561|5562|5563"

# Expected output:
tcp LISTEN 127.0.0.1:5555   # REQ/REP
tcp LISTEN 127.0.0.1:5561   # Control
tcp LISTEN 127.0.0.1:5562   # Visualization  
tcp LISTEN 127.0.0.1:5563   # REST
```

### **Protocol Validation Testing**

```bash
# Test REST stream protocol validation
python -c "
import zmq, json
ctx = zmq.Context()
sock = ctx.socket(zmq.DEALER)
sock.connect('tcp://localhost:5563')
# Valid REST message
sock.send_multipart([b'', json.dumps({
    'method': 'GET', 
    'route': '/v1/system/health_check'
}).encode()])
print('REST:', json.loads(sock.recv_multipart()[1]))
"

# Test control stream protocol validation
python -c "
import zmq, json
ctx = zmq.Context()
sock = ctx.socket(zmq.DEALER)
sock.connect('tcp://localhost:5561')
# Valid control message
sock.send_multipart([b'', json.dumps({
    'message_type': 'heartbeat'
}).encode()])
print('Control:', json.loads(sock.recv_multipart()[1]))
"
```

### **Performance Testing**

```bash
# Test REST stream performance
python -c "
import asyncio
from godot_bridge.godot_bridge_zmq_rest import ZMQRestClient

async def test_performance():
    client = ZMQRestClient(port=5563)
    await client.connect()
    
    start = time.time()
    for i in range(100):
        await client.get_health_check()
    end = time.time()
    
    print(f'100 REST requests: {end-start:.2f}s')
    print(f'Average latency: {(end-start)/100*1000:.2f}ms')

asyncio.run(test_performance())
"
```

## 🔍 Monitoring and Debugging

### **Stream Health Monitoring**

```python
# Monitor all streams
async def monitor_streams():
    rest_client = ZMQRestClient(port=5563)
    control_client = ControlClient(port=5561)
    
    # Test REST stream
    try:
        await rest_client.get_health_check()
        print("✅ REST stream: OK")
    except Exception as e:
        print(f"❌ REST stream: {e}")
    
    # Test control stream
    try:
        await control_client.send_heartbeat()
        print("✅ Control stream: OK") 
    except Exception as e:
        print(f"❌ Control stream: {e}")
```

### **Debug Logging**

Enable detailed logging for specific streams:

```bash
export FEAGI_LOG_LEVEL=DEBUG
export FEAGI_DEBUG_STREAMS="rest,control"
```

### **Common Issues and Solutions**

**Issue**: REST requests timing out
- **Solution**: Check REST stream (5563) is listening
- **Debug**: `telnet localhost 5563`

**Issue**: Protocol format errors
- **Solution**: Verify message format matches stream requirements
- **Debug**: Check error messages for specific format requirements

**Issue**: Port conflicts
- **Solution**: Use `ss -tuln` to check port availability
- **Debug**: FEAGI automatically finds available ports if configured ports are in use

## 🚚 Migration Guide

### **From Unified Stream Architecture**

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

### **Configuration Migration**

```bash
# Before
export FEAGI_CONTROL_PORT=5561  # Handled both protocols

# After  
export FEAGI_CONTROL_PORT=5561  # Legacy control only
export FEAGI_REST_PORT=5563     # REST API only
```

## 🎯 Best Practices

### **Stream Selection Guidelines**

Use the appropriate stream for each use case:

- **REST Stream (5563)**: Modern APIs, web interfaces, bridges
- **Control Stream (5561)**: Agent registration, heartbeats, legacy control
- **Visualization Stream (5562)**: Real-time monitoring, dashboards
- **REQ/REP Stream (5555)**: Command-line tools, simple scripts

### **Client Implementation Guidelines**

1. **Use Protocol-Specific Clients**: Don't mix protocols in a single client
2. **Handle Errors Gracefully**: Each stream has specific error formats
3. **Implement Proper Timeouts**: Different streams have different performance characteristics
4. **Monitor Connection Health**: Implement reconnection logic for long-running clients

### **Performance Optimization**

1. **Choose the Right Stream**: Use the most appropriate stream for your use case
2. **Batch Operations**: Where possible, batch multiple operations
3. **Connection Pooling**: Reuse connections for multiple operations
4. **Async Operations**: Use async clients for better throughput

## 📚 Additional Resources

- **ZMQ Documentation**: [https://zeromq.org/](https://zeromq.org/)
- **FEAGI REST API Reference**: `/docs/api/rest_api.md`
- **Performance Benchmarks**: `/docs/benchmarks/zmq_performance.md`
- **Client Examples**: `/examples/zmq_clients/`

---

This architecture implements **Option 4: Separate Dedicated Streams** as planned, providing the cleanest, most maintainable, and highest-performance ZMQ communication solution for FEAGI. 