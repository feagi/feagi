# Godot Bridge Integration Architecture

## Overview

The Godot Bridge serves as a critical middleware component that enables real-time brain visualization by bridging FEAGI's neural simulation data with the Godot game engine. This document outlines the complete architecture, data flow patterns, and integration requirements discovered through comprehensive system analysis.

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Data Flow Pipeline](#data-flow-pipeline)
3. [Communication Protocols](#communication-protocols)
4. [API Integration Patterns](#api-integration-patterns)
5. [Data Transformation Requirements](#data-transformation-requirements)
6. [Critical Integration Points](#critical-integration-points)
7. [Error Handling & Debugging](#error-handling--debugging)
8. [Performance Considerations](#performance-considerations)
9. [Future Enhancements](#future-enhancements)

## System Architecture

### Components Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   FEAGI Core    │    │  Godot Bridge   │    │  Godot Client   │
│                 │    │                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │ REST API    │◄┼────┼►│ ZMQ Client  │ │    │ │ WebSocket   │ │
│ │ (HTTP/ZMQ)  │ │    │ │             │ │    │ │ Client      │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
│                 │    │        │        │    │        │        │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │ ZMQ Server  │◄┼────┼►│ Data Trans- │ │    │ │ Genome      │ │
│ │             │ │    │ │ formation   │ │    │ │ Loader      │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
│                 │    │        │        │    │        │        │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │ Core API    │ │    │ │ WebSocket   │◄┼────┼►│ Brain       │ │
│ │ Service     │ │    │ │ Server      │ │    │ │ Visualizer  │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Core Responsibilities

#### FEAGI Core
- **Neural Simulation**: Real-time brain simulation and processing
- **API Services**: RESTful endpoints for genome, cortical areas, and brain data
- **State Management**: Brain readiness, health monitoring, and configuration

#### Godot Bridge (`godot_bridge_zmq_rest.py`)
- **Protocol Translation**: ZMQ ↔ WebSocket communication bridge
- **Data Transformation**: FEAGI format → Godot-compatible format
- **Real-time Relay**: Live neuron activation and health data streaming
- **Connection Management**: WebSocket client lifecycle and reconnection

#### Godot Client
- **Genome Loading**: Processes cortical area geometry and structure data
- **Visualization**: Real-time 3D brain visualization and interaction
- **User Interface**: Brain monitoring, configuration, and control panels

## Data Flow Pipeline

### 1. Initialization Sequence

```
Startup Flow:
1. Godot Bridge starts and connects to FEAGI ZMQ (port 5555)
2. Godot Bridge starts WebSocket server (port 9053)
3. Godot Client connects to Bridge WebSocket server
4. Bridge monitors FEAGI health endpoint (/v1/system/health)
5. On brain_readiness: true → Bridge triggers cortical area data fetch
```

### 2. Cortical Area Data Flow

```
FEAGI → Bridge → Godot:

1. FEAGI: GET /v1/cortical_area/cortical_area/geometry
   Returns: { "cortical_id1": {...}, "cortical_id2": {...} }

2. Bridge: Data transformation (FEAGI → Godot format)

3. Bridge → Godot: WebSocket transmission
   Format: {"cortical_areas": [{"id": "...", "name": "...", ...}]}

4. Godot: Genome loading via FEAGI_load_all_cortical_areas()
   Creates: AbstractCorticalArea objects in FeagiCore cache
```

### 3. Real-time Data Streaming

```
Continuous Flow:
1. FEAGI generates neuron activation data
2. Bridge monitors /v1/system/health for brain state changes
3. Bridge relays real-time data via WebSocket
4. Godot processes visualization updates
```

## Communication Protocols

### ZMQ Communication (FEAGI ↔ Bridge)

**Connection**: `tcp://localhost:5555`
**Pattern**: Request-Reply

```python
# Request Format
{
    'route': '/v1/cortical_area/cortical_area/geometry',
    'method': 'GET',
    'body': {},
    'query': {},
    'headers': {}
}

# Response Format
{
    'status_code': 200,
    'body': { "cortical_id": {...}, ... },
    'headers': {'Content-Type': 'application/json'}
}
```

### WebSocket Communication (Bridge ↔ Godot)

**Connection**: `ws://localhost:9053`
**Pattern**: Publisher-Subscriber

```javascript
// Cortical Area Data
{
    "cortical_areas": [
        {
            "id": "CIHMot",
            "name": "M1_BW",
            "dimensions": [10, 10, 1],
            "position": [0, 0, 0],
            "type": "CUSTOM",
            "properties": {
                "cortical_id": "CIHMot",
                "cortical_name": "M1_BW",
                "coordinates_3d": [-45, 0, -14],
                // ... complete cortical properties
            }
        }
    ]
}
```

### Health Monitoring

**Endpoint**: `GET /v1/system/health`
**Frequency**: Every 1 second
**Critical Fields**:
- `brain_readiness`: Triggers cortical area data transmission
- `genome_availability`: Indicates genome load status
- `burst_engine`: Neural processing state

## API Integration Patterns

### 1. FEAGI API Endpoints Used

#### Primary Data Endpoints
- `GET /v1/cortical_area/cortical_area/geometry` - **Core cortical structure data**
- `GET /v1/system/health` - **Brain state monitoring**
- `POST /v1/cortical_area/multi/cortical_area_properties` - **Batch cortical properties**

#### Supporting Endpoints
- `GET /v1/cortical_area/cortical_area_id_list` - Cortical area enumeration
- `GET /v1/region/regions_members` - Brain region hierarchy
- `GET /v1/morphology/morphologies` - Neural morphologies
- `GET /v1/cortical_area/cortical_map_detailed` - Connectivity mappings

### 2. Data Format Requirements

#### FEAGI API Response Format (Fixed in our implementation)
```python
# /v1/cortical_area/cortical_area/geometry NOW returns:
{
    "cortical_id1": {
        "cortical_id": "CIHMot",
        "cortical_name": "M1_BW",
        "cortical_group": "interconnect",
        "coordinates_3d": [-45, 0, -14],
        "cortical_dimensions": [10, 10, 1],
        "visualization": true,
        # ... complete properties
    },
    "cortical_id2": { ... }
}
```

#### Godot Expected Format
```python
# Godot processes this via FEAGI_load_all_cortical_areas()
{
    "cortical_id": {
        "cortical_id": "string",
        "cortical_name": "string",
        "cortical_group": "string",      # → CORTICAL_AREA_TYPE enum
        "cortical_sub_group": "string",
        "coordinates_3d": [x, y, z],     # → Vector3i
        "cortical_dimensions": [w, h, d], # → Vector3i
        "coordinates_2d": [x, y],        # → Vector2i (optional)
        "visualization": boolean,
        # ... neuron firing parameters
    }
}
```

## Data Transformation Requirements

### Critical Field Mappings

| FEAGI Field | Godot Field | Type | Notes |
|-------------|-------------|------|-------|
| `cortical_id` | `cortical_id` | string | **Must match exactly** |
| `cortical_name` | `cortical_name` | string | **Must match exactly** |
| `coordinates_3d` | `coordinates_3d` | `[x,y,z]` | Array of integers |
| `cortical_dimensions` | `cortical_dimensions` | `[w,h,d]` | Array of integers |
| `cortical_group` | `cortical_group` | string | Maps to `CORTICAL_AREA_TYPE` |
| `visualization` | `visualization` | boolean | Controls visibility |

### Legacy Format Transformation

The bridge performs critical format transformation:

```python
# Bridge transformation (simplified)
def transform_cortical_data(feagi_data):
    godot_areas = []
    for cortical_id, area_data in feagi_data.items():
        godot_area = {
            "id": area_data["cortical_id"],           # Godot expects "id"
            "name": area_data["cortical_name"],       # Godot expects "name"
            "dimensions": area_data["cortical_dimensions"],
            "position": area_data["coordinates_3d"],
            "type": map_cortical_type(area_data["cortical_group"]),
            "properties": area_data  # Full FEAGI data in properties
        }
        godot_areas.append(godot_area)

    return {"cortical_areas": godot_areas}
```

## Critical Integration Points

### 1. Genome Loading Synchronization

**Issue**: Godot's genome loading expects complete, consistent data sets.

**Solution**:
- Bridge waits for `brain_readiness: true` before data transmission
- Immediate data send on WebSocket connection if brain already ready
- Atomic data transmission (all cortical areas in single message)

### 2. Data Consistency Enforcement

**Issue**: Different transport protocols (HTTP vs ZMQ) were returning different data.

**Solution**:
- Single Source of Truth API pattern in FEAGI v1 endpoints
- All transports delegate to identical v1 API methods
- Eliminated custom transport-specific implementations

### 3. Real-time State Management

**Challenge**: Coordinating brain state across FEAGI, Bridge, and Godot.

**Implementation**:
- Health monitoring with 1-second polling
- WebSocket connection lifecycle management
- Automatic reconnection and data resynchronization

## Error Handling & Debugging

### Common Issues and Solutions

#### 1. "Unknown" Values in Cortical Data
**Cause**: Field name mismatches (`id` vs `cortical_id`)
**Fix**: Ensure FEAGI API returns correct field names
**Debug**: Check raw WebSocket data for field presence

#### 2. No Cortical Visualization
**Cause**: Data format incompatibility with Godot genome loader
**Fix**: Verify dictionary structure matches `FEAGI_load_all_cortical_areas()` expectations
**Debug**: Enable Godot cache logging for object creation

#### 3. ZMQ Connection Failures
**Cause**: FEAGI not started or port conflicts
**Fix**: Verify FEAGI running on port 5555, check port availability
**Debug**: Monitor ZMQ socket connection status

#### 4. WebSocket Disconnections
**Cause**: Network issues or Godot client crashes
**Fix**: Implement automatic reconnection with exponential backoff
**Debug**: WebSocket connection state logging

### Debugging Tools

#### Bridge Logging
```python
# Comprehensive debug logging implemented
logger.info(f"🔄 Data size to queue: {len(data)} bytes")
logger.info(f"📤 Raw data content (first 500 chars): {data[:500]}")
logger.info(f"✅ Connected clients: {len(connected_clients)}")
```

#### Packet Tracking
```python
# Added cumulative statistics
statistics = {
    'packets_sent_total': cumulative_counter,
    'packets_sent_session': session_counter,
    'websocket_connections': active_connections,
    'brain_readiness': current_state
}
```

## Performance Considerations

### 1. Data Volume Optimization
- **Cortical Area Data**: ~28KB typical payload
- **Real-time Updates**: Minimize redundant transmissions
- **Compression**: Consider WebSocket compression for large datasets

### 2. Connection Management
- **Connection Pooling**: Reuse ZMQ connections to FEAGI
- **WebSocket Efficiency**: Binary frames for high-frequency data
- **Backpressure Handling**: Queue management for slow Godot clients

### 3. Memory Management
- **Data Caching**: Cache cortical structure data to avoid repeated API calls
- **Garbage Collection**: Proper cleanup of WebSocket connections
- **State Synchronization**: Efficient differential updates

## Future Enhancements

### 1. Protocol Improvements
- **Binary Protocols**: More efficient data serialization (Protocol Buffers/MessagePack)
- **Compression**: Real-time data compression for bandwidth optimization
- **Multiplexing**: Single connection handling multiple data streams

### 2. Scalability Enhancements
- **Multi-client Support**: Enhanced support for multiple Godot instances
- **Load Balancing**: Distribute visualization load across clients
- **Horizontal Scaling**: Bridge clustering for high-availability

### 3. Integration Features
- **Authentication**: Secure client authentication and authorization
- **Configuration Management**: Dynamic bridge configuration without restart
- **Health Monitoring**: Enhanced diagnostics and performance metrics

### 4. Developer Experience
- **API Documentation**: OpenAPI/Swagger specs for bridge APIs
- **Testing Framework**: Automated integration tests for FEAGI-Bridge-Godot pipeline
- **Development Tools**: Bridge development and debugging utilities

## Conclusion

The Godot Bridge architecture represents a sophisticated middleware solution that successfully bridges the gap between FEAGI's modern neural simulation capabilities and Godot's powerful 3D visualization engine. The key to its success lies in:

1. **Precise Data Transformation**: Converting between FEAGI's API format and Godot's expected data structures
2. **Robust Communication**: Reliable ZMQ and WebSocket protocol handling
3. **State Synchronization**: Coordinated brain state management across all components
4. **Error Resilience**: Comprehensive error handling and debugging capabilities

The architecture demonstrates the importance of understanding both ends of an integration deeply - from FEAGI's API patterns to Godot's genome loading requirements - to create a seamless, high-performance brain visualization system.

---

**Document Version**: 1.0
**Last Updated**: 2025-05-24
**Contributors**: System Analysis Team
**Related Documents**:
- [API Decorator Architecture](arch-api-decorator-architecture.md)
- [ZMQ Architecture](arch-zmq.md)
- [System Overview](arch-system-overview.md)
