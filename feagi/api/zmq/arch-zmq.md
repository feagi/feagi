# ZeroMQ Architecture for FEAGI 2.0

## Overview

This document outlines the high-performance, production-ready ZeroMQ (ZMQ) communication architecture for FEAGI 2.0, featuring zero-copy neural data paths, real-time sensor data support, and complete Rust/RTOS migration readiness.

## Core Design Principles

### 🚀 Maximum Performance
- **Zero-copy data paths** for neural data streams
- **Direct binary protocols** with minimal overhead
- **Pre-converted neural data** from FEAGI_Connector
- **Lock-free data structures** where possible
- **NUMA-aware memory allocation** for multi-socket systems

### 🛡️ True Modularity
- **Clean separation** between sensor processing (FEAGI_Connector) and neural processing (FEAGI)
- **Protocol/transport separation** via plugin architecture
- **Dynamic protocol discovery** without recompilation
- **Platform abstraction layer** for OS-specific optimizations

### 🦀 Rust/RTOS Ready
- **Fixed-size data structures** throughout
- **No dynamic allocation** in critical paths
- **Error codes** instead of exceptions
- **Deterministic timing** guarantees
- **Static memory pools** with compile-time sizing

### 🖥️ Platform Agnostic
- **Unified API** across Windows/Linux/macOS
- **Platform-specific optimizations** under the hood
- **Conditional compilation** for platform features
- **Abstracted I/O operations** for portability

## Communication Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        External Sensor Processing                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                      FEAGI_Connector (Separate Process)              │    │
│  │  • Camera/LiDAR → Neural Data Conversion                            │    │
│  │  • Video Decoding (H.264/H.265)                                     │    │
│  │  • Sensor Fusion & Preprocessing                                    │    │
│  │  • Neural Encoding (sensor data → neural firing patterns)           │    │
│  │  • feagi_bytes Protocol Encoding                                    │    │
│  └───────────────────────────────┬─────────────────────────────────────┘    │
│                                  │ Neural Data (feagi_bytes)                 │
│                                  ▼                                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                           FEAGI ZMQ Architecture                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐                            ┌─────────────┐                  │
│  │   Control   │    REST/JSON Protocol      │  FastAPI    │                  │
│  │   Stream    │ ────────────────────────────│  Server     │                  │
│  │ Port 5561   │    (Low Frequency)         │ Port 8000   │                  │
│  └─────────────┘                            └─────────────┘                  │
│        │                                           │                         │
│        └─────────────────┬─────────────────────────┘                         │
│                          ▼                                                   │
│                ┌─────────────────────────────┐                               │
│                │    Unified REST API v1      │                               │
│                │ • Provisioning & Config     │                               │
│                │ • Status Queries            │                               │
│                │ • Agent Management          │                               │
│                └─────────────────────────────┘                               │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │              High-Performance Neural Data Streams                        │ │
│  ├─────────────┬─────────────────┬─────────────────┬───────────────────────┤ │
│  │  Sensory    │     Motor       │ Visualization   │    Morphology       │ │
│  │  Stream     │     Stream      │    Stream       │    Stream           │ │
│  │ Port 5558   │   Port 9050     │  Port 5562      │   Port 5564         │ │
│  │             │                 │                 │                     │ │
│  │ PULL Socket │   PUB Socket    │  PUB Socket     │  REQ/REP Socket     │ │
│  │ Neural In   │   Neural Out    │  Brain State    │  Structure Updates  │ │
│  └─────────────┴─────────────────┴─────────────────┴───────────────────────┘ │
│        │               │                 │                   │               │
│        ▼               ▼                 ▼                   ▼               │
│  ┌──────────────────────────────────────────────────────────────────────┐    │
│  │              Neural Data Protocol Processing                          │    │
│  │ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ │    │
│  │ │ NEURON_FLAT  │ │ NEURON_SPARSE│ │ NEURON_MULTI │ │ CORTICAL_MAP │ │    │
│  │ │ Dense Arrays │ │ Compressed   │ │ Multi-Modal  │ │ Topology     │ │    │
│  │ └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘ │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
│                          │                                                   │
│                          ▼                                                   │
│  ┌──────────────────────────────────────────────────────────────────────┐    │
│  │                    Zero-Copy Memory Management                        │    │
│  │ • Ring Buffers    • Memory Pools    • NUMA Aware    • Lock-Free      │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
│                          │                                                   │
│                          ▼                                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                           FEAGI Core Engine                                    │
│  • CoreAPIService    • FCL Injection    • Neural Processing    • BDU         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Architecture

### Sensor Data Processing Flow

```
1. External Sensors (Camera, LiDAR, etc.)
        ↓
2. FEAGI_Connector (Separate Application)
   • Decode video streams (H.264/H.265)
   • Process raw sensor data
   • Convert to neural firing patterns
   • Encode as feagi_bytes
        ↓
3. FEAGI Sensory Stream (Port 5558)
   • Receive neural data bytes
   • Zero-copy decode to neural arrays
   • Pass to CoreAPIService
        ↓
4. CoreAPIService
   • FCL (FEAGI Cortical Layer) injection
   • Neural processing
        ↓
5. Motor/Visualization Output
```

## Stream Specifications

### 1. Control Stream (REST/JSON Protocol)

**Purpose**: Low-frequency control operations, configuration, and status queries
**Transport**: ZMQ ROUTER/DEALER
**Protocol**: JSON-based REST API v1
**Use Cases**: Agent registration, parameter updates, health checks

```python
class ControlStream:
    """REST/JSON control stream for management operations."""
    
    def __init__(self, api_v1_handler: V1APIHandler):
        self.api_v1 = api_v1_handler  # Shared with FastAPI
        self.socket = zmq.Context().socket(zmq.ROUTER)
        self.socket.bind("tcp://*:5561")
    
    async def handle_control_request(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """Process control messages using unified REST API."""
        # REST is perfect for control operations - no performance impact
        return await self.api_v1.handle_request(
            method=message.get("method"),
            path=message.get("path"),
            params=message.get("params"),
            body=message.get("body")
        )
```

### 2. Sensory Stream (Neural Data Protocol)

**Purpose**: High-throughput neural data ingestion from FEAGI_Connector
**Transport**: ZMQ PULL Socket
**Protocol**: feagi_bytes with pre-converted neural data
**Performance**: Up to 10 Gbps neural data throughput

```python
@dataclass
class NeuralDataHeader:
    """Fixed-size header for neural data packets."""
    protocol_id: uint8      # 1 byte (NEURON_FLAT, NEURON_SPARSE, etc.)
    version: uint8          # 1 byte
    flags: uint16          # 2 bytes
    timestamp: uint64      # 8 bytes (nanoseconds)
    cortical_area_id: uint32 # 4 bytes
    neuron_count: uint32    # 4 bytes
    payload_size: uint32    # 4 bytes
    sequence_number: uint64 # 8 bytes
    # Total: 32 bytes fixed header

class SensoryStream:
    """High-performance neural data ingestion from FEAGI_Connector."""
    
    def __init__(self, core_api: CoreAPI):
        self.core_api = core_api
        self.socket = zmq.Context().socket(zmq.PULL)
        self.socket.setsockopt(zmq.RCVHWM, 1000)  # Backpressure
        
        # Platform-specific optimizations
        if sys.platform == "linux":
            self.socket.setsockopt(zmq.SO_ZEROCOPY, 1)
        
        # Pre-allocated ring buffer for zero-copy
        self.ring_buffer = ZeroCopyRingBuffer(
            slots=1024,
            slot_size=1048576  # 1MB per slot for large neural arrays
        )
        
        # Neural data decoders
        self.decoders = {
            ByteStructureID.NEURON_FLAT: self._decode_neuron_flat,
            ByteStructureID.NEURON_SPARSE: self._decode_neuron_sparse,
            ByteStructureID.NEURON_MULTI: self._decode_multi_modal,
        }
    
    def process_neural_data(self) -> StreamResult:
        """Zero-copy neural data processing."""
        # Receive directly into ring buffer
        slot = self.ring_buffer.get_write_slot()
        if not slot:
            return StreamResult.BUFFER_FULL
        
        try:
            # Zero-copy receive into pre-allocated memory
            nbytes = self.socket.recv_into(slot.memory_view, zmq.NOBLOCK)
            
            # Parse fixed-size header without allocation
            header = NeuralDataHeader.from_buffer(slot.memory_view[:32])
            
            # Decode neural data based on protocol
            decoder = self.decoders.get(header.protocol_id)
            if decoder:
                neural_data = decoder(header, slot.memory_view[32:32+header.payload_size])
                
                # Direct injection to FCL without copying
                return self.core_api.inject_neural_data(
                    cortical_area_id=header.cortical_area_id,
                    neural_data=neural_data,
                    timestamp=header.timestamp
                )
            else:
                return StreamResult.UNKNOWN_PROTOCOL
                
        except zmq.Again:
            return StreamResult.NO_DATA
        finally:
            self.ring_buffer.commit_write(slot)
    
    def _decode_neuron_flat(self, header: NeuralDataHeader, 
                           data: memoryview) -> NeuralData:
        """Decode dense neural array format."""
        # Zero-copy view into neural firing data
        neuron_count = header.neuron_count
        
        # Create views without copying
        return NeuralData(
            firing_rates=np.frombuffer(data[0:neuron_count*4], dtype=np.float32),
            coordinates=np.frombuffer(data[neuron_count*4:], dtype=np.int32).reshape(-1, 3)
        )
```

### 3. Motor Stream (Neural Data Protocol)

**Purpose**: Real-time motor command distribution
**Transport**: ZMQ PUB Socket  
**Protocol**: Compact neural command format
**Performance**: < 1ms latency

```python
class MotorStream:
    """Real-time neural motor command broadcasting."""
    
    def __init__(self, core_api: CoreAPI):
        self.core_api = core_api
        self.socket = zmq.Context().socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 100)  # Prevent buffer bloat
        
        # Pre-allocated command buffers
        self.buffer_pool = FixedBufferPool(
            count=256,
            size=65536,  # 64KB for motor commands
            alignment=64  # Cache line aligned
        )
    
    def publish_motor_commands(self) -> StreamResult:
        """Zero-allocation motor command publishing."""
        buffer = self.buffer_pool.acquire()
        if not buffer:
            return StreamResult.BUFFER_EXHAUSTED
        
        try:
            # Get motor neural data without allocation
            motor_data = self.core_api.get_motor_neural_output(buffer.data)
            
            if motor_data.has_commands:
                # Build header in-place
                header = NeuralDataHeader(
                    protocol_id=ByteStructureID.NEURON_FLAT,
                    timestamp=self.get_monotonic_timestamp(),
                    cortical_area_id=motor_data.cortical_id,
                    neuron_count=motor_data.neuron_count,
                    payload_size=motor_data.data_size
                )
                header.write_to(buffer.data)
                
                # Zero-copy send
                self.socket.send(buffer.data[:32+motor_data.data_size], copy=False)
                
            return StreamResult.SUCCESS
            
        finally:
            self.buffer_pool.release(buffer)
```

### 4. Visualization Stream (Neural State Protocol)

**Purpose**: High-bandwidth brain state visualization
**Transport**: ZMQ PUB Socket with topics
**Protocol**: Compressed neural state with adaptive quality
**Performance**: Adaptive rate based on subscribers

```python
class VisualizationStream:
    """Adaptive neural state broadcasting for visualization."""
    
    def __init__(self, core_api: CoreAPI):
        self.core_api = core_api
        self.socket = zmq.Context().socket(zmq.PUB)
        
        # Quality of Service manager
        self.qos = AdaptiveQoSManager(
            min_fps=1,
            max_fps=60,
            target_latency_ms=50
        )
        
        # Compression for large neural states
        self.compressors = {
            CompressionType.NONE: NullCompressor(),
            CompressionType.LZ4: LZ4Compressor(),
            CompressionType.ZSTD: ZstdCompressor(level=3),
        }
    
    def publish_brain_state(self) -> StreamResult:
        """Publish complete brain state for visualization."""
        # Get appropriate detail level based on subscribers
        detail_level = self.qos.get_detail_level()
        
        # Get neural state data
        brain_state = self.core_api.get_brain_state(detail_level)
        
        # Encode neural data
        encoder = ByteStructureEncoder()
        encoded = encoder.encode_neuron_flat({
            'cortical_areas': brain_state.active_areas,
            'neuron_positions': brain_state.positions,
            'firing_rates': brain_state.firing_rates,
            'membrane_potentials': brain_state.potentials,
            'connections': brain_state.connections if detail_level == 'high' else None
        })
        
        # Compress if beneficial
        if len(encoded) > 65536:  # Compress large states
            compressed = self.compressors[CompressionType.LZ4].compress(encoded)
            payload = compressed
            compression_flag = CompressionType.LZ4
        else:
            payload = encoded
            compression_flag = CompressionType.NONE
            
        # Topic-based publishing
        topic = f"brain.{detail_level}.{compression_flag}"
        self.socket.send_multipart([topic.encode(), payload])
        
        return StreamResult.SUCCESS
```

## Zero-Copy Memory Management

### Ring Buffer Implementation

```python
class ZeroCopyRingBuffer:
    """Lock-free ring buffer for zero-copy neural data."""
    
    def __init__(self, slots: int, slot_size: int):
        # Single contiguous allocation
        self.buffer = mmap.mmap(-1, slots * slot_size)
        self.slots = slots
        self.slot_size = slot_size
        
        # Atomic indices
        self.write_index = multiprocessing.Value('Q', 0)
        self.read_index = multiprocessing.Value('Q', 0)
        
    def get_write_slot(self) -> Optional[BufferSlot]:
        """Get next available write slot without blocking."""
        write_idx = self.write_index.value
        read_idx = self.read_index.value
        
        # Check if buffer is full
        next_write = (write_idx + 1) % self.slots
        if next_write == read_idx:
            return None  # Buffer full
        
        # Return memory view of slot
        offset = write_idx * self.slot_size
        return BufferSlot(
            index=write_idx,
            memory_view=memoryview(self.buffer)[offset:offset+self.slot_size]
        )
```

### Neural Data Buffer Pool

```python
class NeuralBufferPool:
    """Specialized buffer pool for neural data arrays."""
    
    def __init__(self, cortical_config: Dict[str, Any]):
        # Pre-allocate based on cortical area sizes
        self.pools = {}
        
        for area_id, config in cortical_config.items():
            neuron_count = config['neuron_count']
            buffer_size = neuron_count * 4 * 2  # float32 firing + coordinates
            
            self.pools[area_id] = FixedBufferPool(
                count=32,  # 32 buffers per cortical area
                size=buffer_size,
                alignment=64,
                numa_node=config.get('numa_node', 0)
            )
    
    def get_buffer_for_area(self, area_id: str) -> Optional[Buffer]:
        """Get pre-sized buffer for specific cortical area."""
        pool = self.pools.get(area_id)
        return pool.acquire() if pool else None
```

## Protocol Specifications

### Neural Data Protocols

```python
class ByteStructureID:
    """Neural data protocol identifiers."""
    NEURON_FLAT = 10      # Dense neural arrays
    NEURON_SPARSE = 11    # Sparse neural data (compressed)
    NEURON_MULTI = 12     # Multi-modal neural data
    CORTICAL_MAP = 13     # Cortical topology updates
    NEURAL_BURST = 14     # High-frequency burst data

@dataclass
class NeuronFlatProtocol:
    """Dense neural array protocol."""
    
    @staticmethod
    def encode(neural_data: NeuralData) -> bytes:
        """Encode neural data to flat byte array."""
        # Header (32 bytes) + data
        header = NeuralDataHeader(
            protocol_id=ByteStructureID.NEURON_FLAT,
            neuron_count=len(neural_data.firing_rates),
            cortical_area_id=neural_data.area_id,
            timestamp=neural_data.timestamp
        )
        
        # Efficient packing
        buffer = bytearray(32 + len(neural_data.firing_rates) * 16)
        header.write_to(buffer[:32])
        
        # Neural data
        offset = 32
        np.copyto(buffer[offset:], neural_data.firing_rates.tobytes())
        
        return bytes(buffer)
```

## Platform-Specific Optimizations

```python
class PlatformOptimizer:
    """Platform-specific performance optimizations."""
    
    @staticmethod
    def optimize_for_neural_data(socket: zmq.Socket):
        """Optimize socket for neural data transmission."""
        
        if sys.platform == "linux":
            # Linux optimizations
            socket.setsockopt(zmq.SO_ZEROCOPY, 1)
            socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            
            # Set CPU affinity for neural processing
            socket.setsockopt(socket.SOL_SOCKET, socket.SO_INCOMING_CPU, 
                            get_neural_processor_cpu())
                
        elif sys.platform == "darwin":
            # macOS optimizations
            socket.setsockopt(socket.SOL_SOCKET, socket.SO_NOSIGPIPE, 1)
            socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8388608)  # 8MB
            
        elif sys.platform == "win32":
            # Windows optimizations
            socket.setsockopt(zmq.FD, socket.fileno())
            # Enable receive buffer scaling
            socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8388608)
```

## Flow Control and Backpressure

### Neural Data Flow Control

```python
class NeuralFlowController:
    """Flow control for high-throughput neural data."""
    
    def __init__(self):
        self.rate_limiter = TokenBucket(
            capacity=1000,  # messages
            refill_rate=10000  # per second
        )
        
        # Per-cortical-area flow control
        self.area_limiters = {}
        
    def should_accept_neural_data(self, area_id: str, size: int) -> bool:
        """Check if we can accept more neural data."""
        # Global rate limit
        if not self.rate_limiter.consume(1):
            return False
        
        # Per-area limit (prevent one area from flooding)
        area_limiter = self.area_limiters.get(area_id)
        if area_limiter and not area_limiter.consume(size):
            return False
        
        return True
```

## Current Implementation Status

### ✅ Implemented Components

1. **Basic ZMQ Streams**
   - ✅ Control stream with REST/JSON protocol
   - ✅ Sensory stream with basic PULL socket
   - ✅ Motor stream with basic PUB socket
   - ✅ Visualization stream with basic PUB socket

2. **Protocol Support**
   - ✅ feagi_bytes basic encoding/decoding
   - ✅ NEURON_FLAT protocol
   - ✅ Basic header parsing

3. **Configuration**
   - ✅ TOML-based configuration
   - ✅ Environment variable overrides
   - ✅ Basic host/port management

### ❌ Major Gaps to Address

1. **Performance Optimizations**
   - ❌ Zero-copy implementation for neural data
   - ❌ Lock-free data structures
   - ❌ Platform-specific optimizations
   - ❌ NUMA-aware buffer allocation
   - ❌ Ring buffer implementation

2. **Neural Data Handling**
   - ❌ Efficient neural array encoding/decoding
   - ❌ Sparse neural data support
   - ❌ Multi-modal neural data protocol
   - ❌ Cortical topology updates

3. **Memory Management**
   - ❌ Pre-sized buffers per cortical area
   - ❌ Neural-specific buffer pools
   - ❌ Memory alignment for SIMD operations
   - ❌ Zero-allocation in critical path

4. **Flow Control**
   - ❌ Backpressure for high-frequency neural data
   - ❌ Per-cortical-area rate limiting
   - ❌ Adaptive flow control
   - ❌ Priority-based scheduling

5. **Rust/RTOS Compatibility**
   - ❌ Fixed-size neural data structures
   - ❌ C-compatible neural data headers
   - ❌ Deterministic memory allocation
   - ❌ Error codes throughout

6. **Platform Abstractions**
   - ❌ SIMD optimizations for neural operations
   - ❌ CPU affinity for neural processing
   - ❌ NUMA-aware thread placement
   - ❌ Platform-specific buffer sizes

### 🚧 Migration Path

#### Phase 1: Neural Data Optimization (Q1 2025)
1. Implement zero-copy for neural data streams
2. Create cortical-area-specific buffer pools
3. Add ring buffers for sensory stream
4. Optimize neural data encoding/decoding

#### Phase 2: Advanced Protocols (Q2 2025)
1. Implement sparse neural data protocol
2. Add multi-modal neural support
3. Create cortical topology protocol
4. Implement burst mode for high-frequency data

#### Phase 3: Rust Migration Prep (Q3 2025)
1. Convert to C-compatible headers
2. Create fixed-size neural structures
3. Implement deterministic allocation
4. Remove Python-specific patterns

#### Phase 4: Full Rust Implementation (Q4 2025)
1. Port neural data handling to Rust
2. Implement SIMD optimizations
3. Create RTOS-compatible version
4. Performance validation

### 📊 Performance Targets

| Metric | Current | Target | Gap |
|--------|---------|--------|-----|
| Neural Data Latency | 10-15ms | <1ms | 10x improvement needed |
| Throughput (neurons/sec) | 1M | 100M | 100x improvement needed |
| Memory Allocation | Dynamic | Static | Complete redesign |
| CPU Usage | 15-20% | <5% | Optimization needed |
| Cortical Areas Supported | 10 | 1000+ | Scalability needed |

## Debugging and Performance Analysis

FEAGI 2.0 includes a comprehensive, high-performance debugging system for ZMQ traffic analysis. The debug system provides:

### 🔍 Zero-Overhead Debugging
- **Runtime control** via REST API
- **Zero overhead when disabled** (no function calls)
- **Console or file output** options
- **Performance impact monitoring**

### 📊 Debug Capabilities
- **Message filtering** by type (sensory, motor, visualization, control)
- **Endpoint filtering** for specific connections
- **Rate limiting** to prevent log spam
- **Statistics tracking** for performance analysis
- **Multiple verbosity levels** (off, minimal, headers, summary, full)

### 🚀 Quick Usage Examples

**Enable console debugging for visualization stream:**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -H "Content-Type: application/json" \
  -d '{"outbound_enabled": true, "console_output": true, "message_filters": ["visualization"]}'
```

**Monitor all ZMQ traffic with rate limiting:**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"inbound_enabled": true, "outbound_enabled": true, "debug_level": "summary", "rate_limit_per_second": 10, "console_output": true}'
```

**Check debug system performance:**
```bash
curl -s http://localhost:8000/v1/debug/zmq/status | jq '.stats'
```

### 📚 Complete Documentation
For comprehensive debugging documentation, see [DEBUG_GUIDE.md](DEBUG_GUIDE.md) which covers:
- Detailed configuration options
- Common debugging scenarios
- Performance considerations
- Troubleshooting guide
- API reference
- Best practices

## Related Documentation

- **[DEBUG_GUIDE.md](DEBUG_GUIDE.md)** - Comprehensive ZMQ debugging guide
- **[README.md](README.md)** - ZMQ architecture overview
- **[../rest/README.md](../rest/README.md)** - REST API documentation
- **[../../config/README.md](../../config/README.md)** - Configuration system

## License

Copyright 2025 Neuraville Inc. Licensed under Apache 2.0. 