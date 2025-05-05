# FEAGI API Architecture Design

## Overview

This document outlines the architecture for FEAGI's API infrastructure, designed to provide both REST API and ZeroMQ (ZMQ) interfaces for programmatic interaction with FEAGI. The design focuses on creating a unified middleware layer that abstracts the communication protocols while ensuring consistency, performance, and future extensibility.

## Goals

- Create a unified API interface that supports both REST and ZMQ protocols
- Design for future migration to Rust
- Ensure backward compatibility where feasible
- Optimize for performance in real-time neural simulation scenarios
- Support versioning to allow for API evolution
- Provide comprehensive documentation and client libraries

## Architecture Overview

```
                      ┌───────────────────────────────────────┐
                      │           Client Applications         │
                      └───────────────────────────────────────┘
                                        │
                                        ▼
            ┌───────────────────────────────────────────────────────┐
            │                   Client Libraries                    │
            │    (Python, JavaScript, Rust, etc. SDK Wrappers)      │
            └───────────────────────────────────────────────────────┘
                      │                                 │
                      ▼                                 ▼
┌────────────────────────────────────┐   ┌────────────────────────────────────┐
│          REST API Interface        │   │          ZMQ Interface             │
│   (FastAPI - HTTP/JSON Endpoints)  │   │   (PUB/SUB, REQ/REP Patterns)      │
└────────────────────────────────────┘   └────────────────────────────────────┘
                      │                                 │
                      ▼                                 ▼
            ┌───────────────────────────────────────────────────────┐
            │              API Gateway / Router Layer                │
            │  (Protocol Translation, Authentication, Rate Limiting) │
            └───────────────────────────────────────────────────────┘
                                        │
                                        ▼
            ┌───────────────────────────────────────────────────────┐
            │               Core API Service Layer                  │
            │     (Resource Handlers, Business Logic Interface)     │
            └───────────────────────────────────────────────────────┘
                                        │
                                        ▼
            ┌───────────────────────────────────────────────────────┐
            │                 FEAGI Core System                     │
            │  (Connectome Manager, Neuroembryogenesis, etc.)       │
            └───────────────────────────────────────────────────────┘
```

## Folder Structure

The API infrastructure is organized using a modular, layered approach that reflects the architecture described above:

```
feagi/
├── api/                       # Main API package
│   ├── __init__.py            # Package exports
│   ├── app.py                 # Main application entry point
│   ├── config.py              # API configuration
│   ├── common/                # Shared utilities across all layers
│   │   ├── __init__.py
│   │   ├── enums.py           # Common enumerations
│   │   ├── errors.py          # Error definitions
│   │   ├── models.py          # Shared data models
│   │   └── utils.py           # Utility functions
│   │
│   ├── core/                  # Core API service layer
│   │   ├── __init__.py
│   │   ├── service.py         # Main CoreApiService class
│   │   ├── connectome.py      # Connectome resource handlers
│   │   ├── neurons.py         # Neuron resource handlers
│   │   ├── simulation.py      # Simulation control handlers
│   │   └── events.py          # Event generation system
│   │
│   ├── gateway/               # API Gateway/Router layer
│   │   ├── __init__.py
│   │   ├── router.py          # Main router implementation
│   │   ├── auth.py            # Authentication/authorization
│   │   ├── validation.py      # Request validation
│   │   └── rate_limit.py      # Rate limiting implementation
│   │
│   ├── rest/                  # REST API interface
│   │   ├── __init__.py
│   │   ├── server.py          # FastAPI server setup
│   │   ├── middleware.py      # FastAPI middleware
│   │   ├── routes/            # API routes by version
│   │   │   ├── __init__.py
│   │   │   ├── v1/            # Version 1 routes
│   │   │   │   ├── __init__.py
│   │   │   │   ├── connectome.py
│   │   │   │   ├── neurons.py
│   │   │   │   └── simulation.py
│   │   │   └── v2/            # Version 2 routes
│   │   │       ├── __init__.py
│   │   │       ├── connectome.py
│   │   │       └── ...
│   │   └── models/            # Pydantic models for requests/responses
│   │       ├── __init__.py
│   │       ├── v1/
│   │       │   ├── __init__.py
│   │       │   ├── connectome.py
│   │       │   └── ...
│   │       └── v2/
│   │           ├── __init__.py
│   │           └── ...
│   │
│   ├── zmq/                   # ZMQ interface
│   │   ├── __init__.py
│   │   ├── server.py          # ZMQ server implementation
│   │   ├── handlers.py        # Message handlers
│   │   ├── serialization.py   # Custom serialization
│   │   └── topics.py          # Topic definitions for PUB/SUB
│   │
│   └── clients/               # Official client libraries
│       ├── __init__.py
│       ├── python/            # Python client
│       │   ├── __init__.py
│       │   ├── client.py      # FeagiClient implementation
│       │   └── async_client.py # Async version
│       └── js/                # JavaScript client (stub)
│           └── index.js
│
├── rust/                      # Rust implementations (future)
│   ├── api/                   # Rust API implementation
│   │   ├── src/
│   │   │   ├── core/          # Core API in Rust
│   │   │   ├── rest/          # REST server in Rust
│   │   │   ├── zmq/           # ZMQ server in Rust
│   │   │   └── lib.rs
│   │   └── Cargo.toml
│   │
│   └── ffi/                   # FFI bindings for Python
│       ├── src/
│       │   └── lib.rs
│       └── Cargo.toml
│
└── tests/                     # Tests
    ├── api/
    │   ├── unit/              # Unit tests
    │   │   ├── test_core.py
    │   │   ├── test_rest.py
    │   │   └── test_zmq.py
    │   ├── integration/       # Integration tests
    │   │   ├── test_rest_api.py
    │   │   └── test_zmq_api.py
    │   └── performance/       # Performance benchmarks
    │       ├── test_rest_perf.py
    │       └── test_zmq_perf.py
    └── clients/               # Client library tests
        └── test_python_client.py
```

## Detailed Component Design

### 1. Core API Service Layer

This is the heart of the API architecture, designed to be protocol-agnostic.

**Key Components:**
- **Resource Managers**: Encapsulate FEAGI resources like cortical areas, neurons, and connections
- **Operation Handlers**: Implement the logic for all API operations
- **State Management**: Manage the state of the FEAGI system
- **Event Generation**: Generate events for real-time notifications

**Implementation Approach:**
```python
# Example structure (will be migrated to Rust)
class CoreApiService:
    def __init__(self, feagi_instance):
        self.feagi = feagi_instance
        
    async def get_cortical_area(self, area_id):
        # Implementation that accesses FEAGI directly
        return self.feagi.connectome_manager.get_cortical_area(area_id)
    
    async def create_cortical_area(self, properties):
        # Business logic implementation
        return self.feagi.connectome_manager.add_cortical_area(**properties)
    
    # More methods for other operations...
```

**Rust Migration Path:**
- Define clear interfaces for all core API functions
- Implement pure computation functions in Rust first
- Gradually replace Python implementations with Rust through FFI
- Eventually migrate the entire service layer to Rust

### 2. API Gateway / Router Layer

Responsible for protocol translation, routing, authentication, and cross-cutting concerns.

**Key Components:**
- **Protocol Adapters**: Translate between different protocols
- **Authentication & Authorization**: Security mechanisms
- **Rate Limiting**: Prevent API abuse
- **Request Validation**: Ensure payload correctness
- **Logging & Monitoring**: Track API usage

**Implementation:**
```python
class ApiGateway:
    def __init__(self, core_api_service):
        self.core_api = core_api_service
        self.auth_manager = AuthenticationManager()
        
    async def route_request(self, endpoint, method, payload, auth_info):
        # 1. Authenticate
        if not self.auth_manager.authenticate(auth_info):
            return {"error": "Unauthorized"}, 401
            
        # 2. Validate
        validation_result = self.validate_request(endpoint, method, payload)
        if not validation_result.valid:
            return {"error": validation_result.error}, 400
            
        # 3. Rate limit check
        if self.rate_limiter.is_limited(auth_info.user_id):
            return {"error": "Rate limit exceeded"}, 429
            
        # 4. Route to appropriate handler
        handler = self.get_handler(endpoint, method)
        result = await handler(payload)
        
        # 5. Transform response if needed
        return self.transform_response(result)
```

#### API Gateway Implementation Details

The API Gateway is implemented in a clean, professional manner:

1. **feagi/api/gateway/api_gateway.py**:
   - Provides a single, centralized implementation
   - Uses the singleton pattern for global access
   - Includes environment-based auto-detection and configuration
   - Implements comprehensive API management features including authentication, authorization, and rate limiting
   - Accessible via `from feagi.api import APIGateway, get_api_gateway`

This implementation follows a clean architectural approach where:
- The gateway acts as a facade for all API interactions
- Features are cleanly separated into different methods
- The code follows professional documentation standards

### 3. REST API Interface

Built with FastAPI to provide HTTP/JSON endpoints.

**Key Features:**
- OpenAPI documentation
- Request validation
- Authentication middleware
- Versioned endpoints

**Example Implementation:**
```python
from fastapi import FastAPI, Depends, HTTPException
from pydantic import BaseModel

app = FastAPI(title="FEAGI API")

class CorticalAreaRequest(BaseModel):
    name: str
    dimensions: tuple[int, int, int]
    position: tuple[int, int, int]
    
@app.get("/v1/cortical_areas/{area_id}")
async def get_cortical_area(area_id: str, api_gateway=Depends(get_api_gateway)):
    result = await api_gateway.route_request(
        endpoint="cortical_areas",
        method="GET",
        payload={"area_id": area_id},
        auth_info=get_auth_info()
    )
    
    if "error" in result:
        raise HTTPException(status_code=result.get("status", 400), detail=result["error"])
    
    return result
```

### 4. ZMQ Interface

Provides high-performance, real-time communication using ZeroMQ.

**Message Patterns:**
- **REQ/REP**: For request-response operations (similar to REST)
- **PUB/SUB**: For real-time events and monitoring
- **PUSH/PULL**: For offloading heavy processing tasks

**Example Implementation:**
```python
import zmq
import zmq.asyncio
import json

class ZmqServer:
    def __init__(self, api_gateway, host="*", req_port=5555, pub_port=5556):
        self.api_gateway = api_gateway
        self.context = zmq.asyncio.Context()
        
        # Request-reply socket
        self.req_socket = self.context.socket(zmq.REP)
        self.req_socket.bind(f"tcp://{host}:{req_port}")
        
        # Publisher socket for events
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://{host}:{pub_port}")
        
    async def start(self):
        """Start processing requests."""
        while True:
            message = await self.req_socket.recv_json()
            
            response = await self.api_gateway.route_request(
                endpoint=message.get("endpoint"),
                method=message.get("method"),
                payload=message.get("payload", {}),
                auth_info=message.get("auth", {})
            )
            
            await self.req_socket.send_json(response)
    
    async def publish_event(self, topic, event_data):
        """Publish event to subscribers."""
        message = json.dumps(event_data)
        await self.pub_socket.send_multipart([topic.encode(), message.encode()])
```

### 5. Client Libraries

Provide simplified interfaces for different programming languages.

**Languages to Support:**
- Python (primary)
- JavaScript/TypeScript
- Rust
- C/C++

**Example Python Client:**
```python
class FeagiClient:
    def __init__(self, host, port, use_zmq=False):
        self.base_url = f"http://{host}:{port}/v0"
        self.use_zmq = use_zmq
        
        if use_zmq:
            self.zmq_context = zmq.Context()
            self.req_socket = self.zmq_context.socket(zmq.REQ)
            self.req_socket.connect(f"tcp://{host}:{port}")
        
    def get_cortical_area(self, area_id):
        if self.use_zmq:
            self.req_socket.send_json({
                "endpoint": "cortical_areas",
                "method": "GET",
                "payload": {"area_id": area_id}
            })
            return self.req_socket.recv_json()
        else:
            response = requests.get(f"{self.base_url}/cortical_areas/{area_id}")
            return response.json()
```

### 4.5 ZMQ Best Practices for Mixed Workloads

When implementing a ZMQ-based API that must handle both streaming sensorimotor data and traditional CRUD operations, several best practices should be followed:

##### Pattern Selection for Different Workloads

1. **Use the Right Pattern for Each Workload**
   - **REQ/REP**: Best for traditional CRUD operations with request-response semantics
   - **PUB/SUB**: Ideal for one-to-many broadcasting of motor commands or event notifications
   - **PUSH/PULL**: Excellent for high-throughput sensory data ingestion
   - **DEALER/ROUTER**: Useful for asynchronous request-response when operations have variable processing times

2. **Separate Socket Types by Concerns**
   - Never mix different message patterns on the same socket
   - Use dedicated sockets for each distinct communication pattern
   - Create semantically meaningful socket pairs for each workload type

```python
# Good practice: Dedicated sockets for different concerns
class FeagiZmqServer:
    def __init__(self):
        # Socket for CRUD operations
        self.crud_socket = self.context.socket(zmq.REP)
        self.crud_socket.bind("tcp://*:5555")
        
        # Socket for incoming sensory data
        self.sensory_socket = self.context.socket(zmq.PULL)
        self.sensory_socket.bind("tcp://*:5556")
        
        # Socket for outgoing motor commands
        self.motor_socket = self.context.socket(zmq.PUB)
        self.motor_socket.bind("tcp://*:5557")
```

##### Socket Management Strategies

1. **Port Allocation**
   - Assign different port numbers to different socket types
   - Use consistent port numbering conventions (e.g., base port + offset)
   - Document port usage clearly for client implementations

2. **Context Sharing**
   - Use a single ZMQ context for all sockets in the same process
   - Create separate contexts only when necessary (e.g., for isolated threading models)

```python
# Shared context with multiple sockets
context = zmq.Context.instance()

# CRUD API socket
crud_socket = context.socket(zmq.REP)
crud_socket.bind("tcp://*:5555")

# Streaming data socket
stream_socket = context.socket(zmq.PULL)
stream_socket.bind("tcp://*:5556")
```

3. **Socket Lifecycle Management**
   - Explicitly close sockets when they're no longer needed
   - Use context termination to ensure all sockets are properly closed
   - Implement proper shutdown sequences to avoid message loss

##### Multi-Threading and Concurrency

1. **Dedicated Processing Threads**
   - Use separate threads for different message patterns
   - Avoid blocking operations in any single socket handler
   - Consider using asyncio for concurrent socket handling

```python
def start_server():
    # Create and start the CRUD operations thread
    crud_thread = threading.Thread(target=handle_crud_operations)
    crud_thread.daemon = True
    crud_thread.start()
    
    # Create and start the sensory data processing thread
    sensory_thread = threading.Thread(target=handle_sensory_data)
    sensory_thread.daemon = True
    sensory_thread.start()
    
    # Create and start the motor output thread
    motor_thread = threading.Thread(target=publish_motor_data)
    motor_thread.daemon = True
    motor_thread.start()
```

2. **Polling Multiple Sockets**
   - Use `zmq.Poller()` to efficiently monitor multiple sockets
   - Prioritize sockets based on workload importance and time sensitivity
   - Set appropriate timeouts to balance responsiveness and CPU usage

```python
def poll_multiple_sockets():
    poller = zmq.Poller()
    poller.register(crud_socket, zmq.POLLIN)
    poller.register(sensory_socket, zmq.POLLIN)
    
    while True:
        socks = dict(poller.poll(timeout=100))
        
        if crud_socket in socks and socks[crud_socket] == zmq.POLLIN:
            handle_crud_message(crud_socket.recv_json())
            
        if sensory_socket in socks and socks[sensory_socket] == zmq.POLLIN:
            handle_sensory_data(sensory_socket.recv())
```

3. **Message Prioritization**
   - Implement priority queues for different message types
   - Process control and configuration messages before high-volume data
   - Use separate processing pipelines for different workload types

##### Message Format Considerations

1. **Protocol Differentiation**
   - Use different serialization formats for different workloads
   - JSON for CRUD operations (human-readable, easier debugging)
   - Binary formats for streaming data (better performance, lower overhead)

2. **Envelope Patterns**
   - Use multipart messages with consistent envelope formats
   - Include message type identifiers in envelopes for routing
   - Consider including version information in message formats

```python
# CRUD operation (JSON-based)
crud_socket.send_multipart([
    b"CRUD",  # Message type identifier
    b"v0",    # API version
    json.dumps({
        "operation": "create",
        "resource": "cortical_area",
        "data": {...}
    }).encode()
])

# Sensory data (binary)
sensory_socket.send_multipart([
    b"SENSORY",  # Message type identifier
    area_id.encode(),
    binary_encoded_data
])
```

##### Resource Management

1. **High Water Marks (HWM)**
   - Set appropriate HWM values for different socket types
   - Use higher limits for streaming data sockets
   - Consider lower limits for CRUD operations to prevent backlog

```python
# Configure socket options based on workload type
# Streaming socket with higher buffer limits
streaming_socket = context.socket(zmq.PUSH)
streaming_socket.setsockopt(zmq.SNDHWM, 10000)  # Higher buffer for streaming

# CRUD socket with lower limits to detect backpressure sooner
crud_socket = context.socket(zmq.REQ)
crud_socket.setsockopt(zmq.SNDHWM, 100)  # Lower buffer for CRUD
```

2. **Backpressure Handling**
   - Implement backpressure detection for high-volume streams
   - Add rate limiting for clients that send too much data
   - Provide feedback mechanisms to adjust transmission rates

3. **Memory Management**
   - Pre-allocate buffers for high-frequency data to avoid allocations
   - Implement object pooling for message containers
   - Monitor memory usage and implement circuit breakers

##### Client Design Recommendations

1. **Separate Client Classes**
   - Provide different client classes for CRUD vs. streaming
   - Allow clients to connect to a subset of available services
   - Create specialized clients for specific use cases

```python
# Specialized clients for different concerns
class FeagiApiClient:
    """Client for CRUD operations and configuration."""
    # REQ/REP pattern for API operations

class FeagiSensoryClient:
    """Client for sending sensory data to FEAGI."""
    # PUSH pattern for efficient data transmission

class FeagiMotorClient:
    """Client for receiving motor output from FEAGI."""
    # SUB pattern for receiving motor commands
```

2. **Connection Management**
   - Implement reconnection logic with exponential backoff
   - Handle server unavailability gracefully
   - Support connection to multiple FEAGI instances for redundancy

##### Monitoring and Debugging

1. **Traffic Monitoring**
   - Implement metrics collection for different socket types
   - Monitor message rates, sizes, and processing times
   - Provide diagnostics for bottleneck identification

2. **Message Logging**
   - Log representative samples of different message types
   - Include timestamp and routing information
   - Provide configurable verbosity levels

```python
def log_message_metrics(socket_name, message_type, size, processing_time):
    """Log message metrics for monitoring."""
    metrics_logger.info(f"{socket_name}:{message_type} Size:{size}B Time:{processing_time}ms")
    
    # Update metrics counters
    metrics["message_count"][socket_name] += 1
    metrics["total_bytes"][socket_name] += size
    metrics["processing_time"][socket_name] += processing_time
```

By following these best practices, the ZMQ infrastructure can efficiently handle both the high-throughput streaming needs of sensorimotor data and the more traditional request-response patterns of CRUD operations within a single coherent system.

### 4.6 Brain Visualization Data Streaming

Streaming brain visualization data presents unique challenges that differ from both CRUD operations and sensorimotor data. Visualization streams need to efficiently represent the dynamic state of potentially millions of neurons and their connections while maintaining responsiveness and limiting bandwidth usage.

##### Visualization Data Types

Brain visualization data typically includes:

1. **Neuron Activity Data**
   - Real-time firing patterns across cortical areas
   - Membrane potential and other neuron state variables
   - Typically sparse (only a fraction of neurons active at once)

2. **Structural Data**
   - Neuron positions and connectivity information
   - Cortical area boundaries and properties
   - Changes less frequently than activity data

3. **Analysis Metrics**
   - Activation heatmaps and intensity distributions
   - Network statistics and activity summaries
   - Region-of-interest (ROI) focused data

##### ZMQ Implementation for Visualization

To efficiently stream visualization data, we implement specialized ZMQ patterns:

```python
# feagi/api/zmq/server.py
class ZmqVisualizationServer:
    def __init__(self, host="*", base_port=5560):
        self.context = zmq.Context.instance()
        
        # Socket for structural data (changes infrequently)
        self.structure_socket = self.context.socket(zmq.PUB)
        self.structure_socket.bind(f"tcp://{host}:{base_port}")
        
        # Socket for real-time activity data (high frequency)
        self.activity_socket = self.context.socket(zmq.PUB)
        self.activity_socket.bind(f"tcp://{host}:{base_port+1}")
        
        # Socket for client requests (view changes, filters, etc.)
        self.control_socket = self.context.socket(zmq.ROUTER)
        self.control_socket.bind(f"tcp://{host}:{base_port+2}")
        
        # Track connected clients and their view settings
        self.clients = {}
```

##### Optimized Visualization Data Streaming

1. **Level of Detail (LOD) System**
   - Stream different detail levels based on client view settings
   - Implement progressive refinement for zooming operations
   - Send full detail only for regions of interest

```python
def send_activity_update(self, brain_state, timestamp):
    """Send activity updates with level-of-detail optimization."""
    # Create base update with minimal data (low detail)
    base_update = {
        "timestamp": timestamp,
        "summary": self._create_activity_summary(brain_state)
    }
    
    # Send the base update to all clients
    self.activity_socket.send_multipart([
        b"activity.base",
        self.serializer.serialize(base_update)
    ])
    
    # For each detail level, send additional data
    for detail_level in range(1, 4):  # LOD levels 1-3
        clients_at_level = [cid for cid, settings in self.clients.items() 
                           if settings.get("detail_level") >= detail_level]
        
        if not clients_at_level:
            continue
            
        # Create detail level specific data
        detail_data = self._create_detail_level(brain_state, detail_level)
        
        self.activity_socket.send_multipart([
            f"activity.detail.{detail_level}".encode(),
            self.serializer.serialize(detail_data)
        ])
    
    # Send ROI-specific high detail data
    for client_id, settings in self.clients.items():
        roi = settings.get("roi")
        if roi:
            roi_data = self._extract_roi_data(brain_state, roi)
            
            self.activity_socket.send_multipart([
                f"activity.roi.{client_id}".encode(),
                self.serializer.serialize(roi_data)
            ])
```

2. **Data Compression Strategies**

```python
class VisualizationSerializer:
    def serialize_activity(self, activity_data, compression_level=1):
        """Serialize neural activity data with optional compression."""
        # Convert to sparse representation
        active_neurons = {}
        
        for area_id, neurons in activity_data.items():
            # Find active neurons (non-zero values)
            active_indices = np.nonzero(neurons)
            active_values = neurons[active_indices]
            
            if len(active_values) > 0:
                active_neurons[area_id] = {
                    "indices": active_indices,
                    "values": active_values
                }
        
        # Serialize using a compact binary format
        serialized = msgpack.packb(active_neurons)
        
        # Apply compression if requested
        if compression_level > 0:
            if compression_level == 1:
                # Fast compression
                serialized = snappy.compress(serialized)
            else:
                # Higher compression ratio but more CPU intensive
                serialized = zlib.compress(serialized, level=compression_level)
                
        return serialized
```

3. **Selective Updates**
   - Send only changes since the last update (delta encoding)
   - Use timestamp-based filtering for temporal coherence
   - Support view-dependent data filtering

```python
class DeltaEncoder:
    def __init__(self):
        self.previous_state = {}
        
    def encode_delta(self, current_state):
        """Encode only the changes since the previous state."""
        delta = {}
        
        for area_id, neurons in current_state.items():
            if area_id not in self.previous_state:
                # New area, include all data
                delta[area_id] = neurons
                continue
                
            # Calculate difference from previous state
            prev_neurons = self.previous_state[area_id]
            diff = neurons - prev_neurons
            
            # Only include non-zero differences
            if np.any(diff):
                # Find indices where values changed
                changed_indices = np.nonzero(diff)
                delta[area_id] = {
                    "indices": changed_indices,
                    "values": diff[changed_indices]
                }
        
        # Update previous state
        self.previous_state = current_state.copy()
        
        return delta
```

##### Client-Side View Control

Clients need to control what visualization data they receive. The implementation includes:

1. **View Settings Protocol**
   - Clients send view specifications to the server
   - Settings include zoom level, region of interest, and visualization filters
   - Server tracks client settings to optimize data transmission

```python
# Example client-side implementation
class FeagiBrainVisClient:
    def __init__(self, host, base_port=5560):
        self.context = zmq.Context.instance()
        self.client_id = str(uuid.uuid4())
        
        # Socket for structural data
        self.structure_socket = self.context.socket(zmq.SUB)
        self.structure_socket.connect(f"tcp://{host}:{base_port}")
        self.structure_socket.setsockopt(zmq.SUBSCRIBE, b"structure")
        
        # Socket for activity data
        self.activity_socket = self.context.socket(zmq.SUB)
        self.activity_socket.connect(f"tcp://{host}:{base_port+1}")
        
        # Set default subscriptions
        self.activity_socket.setsockopt(zmq.SUBSCRIBE, b"activity.base")
        
        # Socket for sending view control messages
        self.control_socket = self.context.socket(zmq.DEALER)
        self.control_socket.setsockopt(zmq.IDENTITY, self.client_id.encode())
        self.control_socket.connect(f"tcp://{host}:{base_port+2}")
        
    def set_view_settings(self, detail_level=1, roi=None, filters=None):
        """Update view settings on the server."""
        settings = {
            "client_id": self.client_id,
            "detail_level": detail_level,
            "roi": roi,
            "filters": filters or {}
        }
        
        # Send view settings to server
        self.control_socket.send_json({
            "type": "view_settings",
            "settings": settings
        })
        
        # Update subscriptions based on detail level
        if detail_level >= 1:
            self.activity_socket.setsockopt(zmq.SUBSCRIBE, 
                                          f"activity.detail.1".encode())
        if detail_level >= 2:
            self.activity_socket.setsockopt(zmq.SUBSCRIBE, 
                                          f"activity.detail.2".encode())
        if detail_level >= 3:
            self.activity_socket.setsockopt(zmq.SUBSCRIBE, 
                                          f"activity.detail.3".encode())
            
        # Subscribe to ROI-specific messages
        if roi:
            self.activity_socket.setsockopt(zmq.SUBSCRIBE, 
                                          f"activity.roi.{self.client_id}".encode())
```

2. **Interactive Visualization Control**
   - Request specific views of neural data
   - Toggle between different visualization modes
   - Zoom and focus on regions of interest

##### Integration with Main ZMQ Architecture

The visualization streaming integrates with the existing ZMQ infrastructure:

```python
# feagi/api/zmq/server.py
class ZmqServer:
    def __init__(self, api_gateway, host="*", 
                 req_port=5555, pub_port=5556,
                 sensory_port=5557, motor_port=5558,
                 vis_base_port=5560):
        # ... existing initialization code ...
        
        # Add visualization server
        self.vis_server = ZmqVisualizationServer(host, vis_base_port)
        
    async def start(self):
        """Start all ZMQ servers."""
        await asyncio.gather(
            self.handle_crud_requests(),
            self.handle_sensory_data(),
            self.vis_server.start(),
            # Other handlers...
        )
    
    async def update_visualization(self, brain_state):
        """Update visualization with current brain state."""
        await self.vis_server.send_activity_update(
            brain_state,
            timestamp=int(time.time() * 1000)
        )
```

##### Performance Considerations

1. **Neuron Sampling**
   - For very large networks, use statistical sampling
   - Implement dynamic sampling rates based on activity levels
   - Apply spatial decimation for distant or overview perspectives

2. **Hardware Acceleration**
   - Leverage GPU for compression and data processing
   - Consider using shared memory for local visualizations
   - Optimize binary serialization for SIMD instructions

3. **Bandwidth Management**
   - Implement rate limiting per client
   - Dynamically adjust update frequency based on network conditions
   - Provide statistics on data volume for monitoring

##### Visualization Data Types in the API Design

The visualization data streaming capabilities are exposed in the overall API:

```python
# In REST API
@app.get("/v1/visualization/snapshot")
async def get_visualization_snapshot(request: Request):
    """Get the current visualization state as a snapshot."""
    brain_state = await get_current_brain_state()
    return JSONResponse(content=brain_state)

@app.websocket("/v0/visualization/stream")
async def websocket_endpoint(websocket: WebSocket):
    """Stream visualization data over WebSocket (alternative to ZMQ)."""
    await websocket.accept()
    
    # Get client settings
    settings = await websocket.receive_json()
    
    # Register with visualization manager
    client_id = await register_visualization_client(websocket, settings)
    
    try:
        while True:
            # Handle incoming messages (view updates)
            data = await websocket.receive_text()
            await handle_visualization_client_message(client_id, data)
    finally:
        await unregister_visualization_client(client_id)
```

This comprehensive approach to brain visualization data streaming ensures that clients can efficiently receive the data they need while minimizing bandwidth usage and maintaining responsiveness. The level-of-detail system, compression strategies, and view-dependent filtering allow for scaling to large brain simulations with millions of neurons while providing interactive visualization capabilities.

## API Versioning Strategy

Versioning is crucial for API evolution while maintaining backward compatibility. The FEAGI API implements a comprehensive versioning strategy across all aspects of the API:

### 1. REST API Versioning

We use URL path versioning as the primary versioning mechanism for the REST API:

```
/v1/cortical_areas       # Version 1 endpoint
/v2/cortical_areas       # Version 2 endpoint
```

**Implementation Details:**

1. **Router Organization:**
   - Separate router modules for each version (`routes/v1/`, `routes/v2/`)
   - Each version has its own set of Pydantic models for request/response schemas
   - Common code shared via inheritance or composition

2. **Version Lifecycle Management:**
   - **Active:** Current recommended version
   - **Deprecated:** Still supported but scheduled for removal
   - **Sunset:** End-of-life date announced
   - **Retired:** No longer available

3. **Version Headers:**
   - Support for version specification via headers as an alternative
   ```
   X-API-Version: 1
   ```

4. **Documentation:**
   - OpenAPI schema includes version in the documentation
   - Deprecation notices clearly indicated

**Example Implementation:**

```python
# feagi/api/rest/server.py
from fastapi import FastAPI, APIRouter

app = FastAPI(title="FEAGI API")

# Import version-specific routers
from .routes.v1 import router as router_v1
from .routes.v2 import router as router_v2

# Mount version routers
app.include_router(router_v1, prefix="/v0")
app.include_router(router_v2, prefix="/v2")

# Add version header handling middleware
@app.middleware("http")
async def version_header_middleware(request, call_next):
    version = request.headers.get("X-API-Version")
    if version:
        # Rewrite request to use the version specified in the header
        request.scope["path"] = f"/v{version}" + request.url.path
    response = await call_next(request)
    return response
```

### 2. ZMQ API Versioning

ZMQ doesn't have HTTP paths, so we handle versioning differently:

1. **Message Envelope Format:**
   ```json
   {
     "version": "1",
     "endpoint": "cortical_areas",
     "method": "GET",
     "payload": {...}
   }
   ```

2. **Handler Selection:**
   - Use the version field to route to appropriate handler
   - Include version in topic strings for PUB/SUB

3. **Socket Isolation:**
   - Optionally use separate ports for major versions
   - Default port handles all versions but encourages newest

**Example Implementation:**

```python
# feagi/api/zmq/handlers.py
class ZmqHandlerRegistry:
    def __init__(self):
        self.handlers = {
            "1": {},  # Version 1 handlers
            "2": {}   # Version 2 handlers
        }
    
    def register_handler(self, version, endpoint, method, handler):
        """Register a handler for a specific version, endpoint, and method."""
        if version not in self.handlers:
            self.handlers[version] = {}
        
        if endpoint not in self.handlers[version]:
            self.handlers[version][endpoint] = {}
            
        self.handlers[version][endpoint][method] = handler
    
    def get_handler(self, version, endpoint, method):
        """Get the handler for the specified version, endpoint, and method."""
        if version not in self.handlers:
            # Fallback to the latest version if specified version doesn't exist
            version = max(self.handlers.keys())
            
        if endpoint not in self.handlers[version]:
            return None
            
        return self.handlers[version][endpoint].get(method)
```

### 3. Core API Versioning

The Core API layer also needs versioning to support multiple interface versions:

1. **Service Interface Versioning:**
   - Different service method signatures for different versions
   - Version-specific model classes

2. **Implementation Approach:**
   ```python
   # feagi/api/core/service.py
   class CoreApiService:
       async def get_cortical_area_v1(self, area_id):
           # V1 implementation
           area = self.feagi.connectome_manager.get_cortical_area(area_id)
           return self._transform_to_v1_response(area)
           
       async def get_cortical_area_v2(self, area_id, include_neurons=False):
           # V2 implementation with extended functionality
           area = self.feagi.connectome_manager.get_cortical_area(area_id)
           response = self._transform_to_v2_response(area)
           
           if include_neurons:
               neurons = self.feagi.connectome_manager.get_neurons_by_area(area_id)
               response["neurons"] = neurons
               
           return response
   ```

### 4. Version Compatibility Mapping

To avoid duplication and simplify maintenance, use compatibility mapping:

```python
# feagi/api/common/version_map.py
VERSION_COMPATIBILITY = {
    # Map older versions to handler functions for newer versions
    "cortical_areas": {
        "GET": {
            # v0 can be handled by the v2 method with default parameters
            "1": lambda params: service.get_cortical_area_v2(
                params["area_id"], 
                include_neurons=False
            ),
            "2": lambda params: service.get_cortical_area_v2(
                params["area_id"], 
                include_neurons=params.get("include_neurons", False)
            )
        }
    }
}
```

### 5. Client Library Versioning

Client libraries must also support API versioning:

1. **Version Specification:**
   ```python
   # Default to latest version
   client = FeagiClient(host="localhost", port=8000)
   
   # Explicitly request v0
   client_v1 = FeagiClient(host="localhost", port=8000, api_version="1")
   ```

2. **Version-Specific Methods:**
   - Newer client versions can support all API versions
   - Methods can be marked as deprecated
   - Implement version-specific method variations

3. **Semantic Versioning:**
   - Client library uses semver (MAJOR.MINOR.PATCH)
   - MAJOR version changes when breaking changes occur
   - Document compatibility matrix

**Example Client Implementation:**

```python
# feagi/api/clients/python/client.py
import warnings

class FeagiClient:
    def __init__(self, host, port, api_version="2"):
        self.base_url = f"http://{host}:{port}/v{api_version}"
        self.api_version = api_version
    
    def get_cortical_area(self, area_id, include_neurons=None):
        """Get a cortical area by ID.
        
        Args:
            area_id: The ID of the cortical area
            include_neurons: Whether to include neurons (v2+ only)
        """
        params = {}
        
        # Version-specific parameter handling
        if self.api_version >= "2" and include_neurons is not None:
            params["include_neurons"] = include_neurons
        elif include_neurons is not None:
            warnings.warn(
                "The 'include_neurons' parameter is not supported in API v0 "
                "and will be ignored.",
                UserWarning
            )
            
        response = requests.get(
            f"{self.base_url}/cortical_areas/{area_id}", 
            params=params
        )
        return response.json()
```

### 6. Versioning Best Practices

1. **Additive Changes Only:**
   - Add new endpoints rather than changing existing ones
   - Add optional parameters with sensible defaults
   - Return additional fields rather than changing existing ones

2. **Clear Documentation:**
   - Document when features were introduced
   - Include sunset dates for deprecated features
   - Provide migration guides between versions

3. **Automated Testing:**
   - Test all supported versions
   - Test version compatibility (old clients with new server)
   - Ensure backward compatibility

4. **Version Lifecycle Policy:**
   - Clearly communicate support policy
   - Minimum 1 year support for deprecated APIs
   - Regular deprecation cycles (e.g., annual)

5. **Feature Flags:**
   - Use feature flags for gradual rollout of new API features
   - Allow early access to new APIs before official version release

## API Operations

Core operations to be supported by both REST and ZMQ interfaces:

### Connectome Management
- List cortical areas
- Create/update/delete cortical areas
- Get neurons in an area
- Add/update/delete neurons
- Create/modify/delete synaptic connections

### Runtime Control
- Start/stop/pause simulation
- Set simulation speed
- Get/set simulation parameters

### Monitoring
- Subscribe to neural activity
- Get real-time statistics
- Monitor resource usage

### Development/Testing
- Run specific tests
- Benchmark performance
- Export/import connectome states

## Considerations for Rust Migration

### Phased Approach

1. **Phase 1: FFI Integration**
   - Implement performance-critical functions in Rust
   - Expose them to Python via PyO3
   - Keep overall architecture in Python

2. **Phase 2: Core Service Migration**
   - Migrate the Core API Service to Rust
   - Maintain Python bindings for backwards compatibility
   - Add pure-Rust clients

3. **Phase 3: Full Migration**
   - Implement protocol servers (REST, ZMQ) in Rust
   - Create a thin Python wrapper for backward compatibility

### Type System Mapping

Define clear mapping between Python and Rust types:

```rust
// Example Rust structs matching Python classes
#[derive(Serialize, Deserialize)]
struct CorticalArea {
    id: String,
    name: String,
    dimensions: (u32, u32, u32),
    position: (i32, i32, i32),
    properties: HashMap<String, Value>,
}

// Python bindings using PyO3
#[pyclass]
struct PyCorticalArea {
    inner: CorticalArea,
}

#[pymethods]
impl PyCorticalArea {
    #[getter]
    fn id(&self) -> String {
        self.inner.id.clone()
    }
    
    // More getters/setters...
}
```

## Pros and Cons

### Pros

1. **Unified API Surface**
   - Consistent behavior across protocols
   - Single source of truth for API operations
   - Easier documentation and maintenance

2. **Protocol Flexibility**
   - REST for wide compatibility and easy debugging
   - ZMQ for high-performance real-time scenarios
   - Easy to add new protocols in the future

3. **Future-Proof Design**
   - Clean abstraction layers enabling gradual Rust migration
   - Versioned API for compatibility
   - Modular architecture allows replacing components

4. **Performance Optimization**
   - ZMQ provides low-latency communication
   - Ability to optimize critical paths with Rust
   - Efficient event streaming

### Cons

1. **Increased Complexity**
   - Multiple abstraction layers
   - Need to maintain protocol consistency
   - More sophisticated deployment setup

2. **Development Overhead**
   - Need to implement and test multiple interfaces
   - Cross-language development challenges
   - More complex testing requirements

3. **Migration Challenges**
   - FFI boundary performance concerns
   - Duplicated code during transition to Rust
   - Need to ensure behavioral consistency

4. **Documentation Burden**
   - Multiple client libraries to document
   - Protocol-specific documentation needed
   - Need to keep docs in sync with implementations

## Implementation Challenges

### 1. State Synchronization

**Challenge:** Keeping state consistent across the API layer.

**Solution:**
- Implement a state change pub/sub system
- Use atomic transactions for related changes
- Include version numbers in responses for optimistic concurrency

### 2. Error Handling Across Protocols

**Challenge:** Consistent error reporting across REST and ZMQ.

**Solution:**
- Define a common error model
- Create protocol-specific error translators
- Include error codes, messages, and remediation hints

### 3. Authentication/Authorization

**Challenge:** Security model that works for both protocols.

**Solution:**
- Token-based authentication (JWT)
- Permissions based on resource types and operations
- Rate limiting per client identity

### 4. Performance Under Load

**Challenge:** Maintaining performance with high throughput.

**Solution:**
- Implement connection pooling
- Use asynchronous processing
- Consider read/write splitting for high-load scenarios
- Optimize serialization/deserialization

### 5. Testing Strategy

**Challenge:** Testing across multiple protocols and languages.

**Solution:**
- Protocol-agnostic test suite at the core layer
- Protocol-specific tests for each interface
- End-to-end tests with real clients
- Performance benchmarks for critical paths

## Next Steps

1. Create detailed API specifications for core resources
2. Implement the Core API Service layer
3. Build REST API interface with FastAPI
4. Implement ZMQ interface for real-time operations
5. Develop Python client library
6. Begin Rust implementation of performance-critical functions
7. Create API documentation and examples
8. Set up continuous integration with protocol-specific tests

## Conclusion

This architecture provides a robust foundation for FEAGI's API needs, supporting both REST and ZMQ interfaces while preparing for future Rust migration. By focusing on strong abstraction boundaries and clear interfaces, the system can evolve while maintaining backward compatibility and optimal performance.