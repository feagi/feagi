# ZMQ Refactoring Plan

## Overview
This document outlines the step-by-step refactoring plan to transform the current ZMQ implementation to match the architecture defined in arch-zmq.md.

## Current State Analysis
- Basic ZMQ streams exist (sensory, motor, visualization, control, rest)
- No zero-copy implementation
- No neural-specific optimizations
- Dynamic memory allocation throughout
- Missing ring buffers and static pools

## Target Architecture
- Zero-copy neural data paths
- Static memory allocation with pre-sized buffers
- Ring buffer implementation for high-throughput
- Neural-specific protocols (NEURON_FLAT, NEURON_SPARSE, etc.)
- Platform-specific optimizations
- RTOS/Rust ready with fixed-size structures

## Phase 1: Core Infrastructure (Week 1-2)

### 1.1 Create Neural Data Structures
```
Location: feagi/api/zmq/neural/
Files to create:
- protocols.py     # Neural data protocol definitions
- headers.py       # Fixed-size header structures
- buffers.py       # Zero-copy buffer management
- ring_buffer.py   # Lock-free ring buffer
```

### 1.2 Implement Memory Management
```
Location: feagi/api/zmq/memory/
Files to create:
- static_pool.py   # Static buffer pools
- numa_allocator.py # NUMA-aware allocation
- alignment.py     # Memory alignment utilities
```

### 1.3 Platform Optimizations
```
Location: feagi/api/zmq/platform/
Files to create:
- optimizer.py     # Platform-specific socket optimization
- linux_opt.py     # Linux-specific (SO_ZEROCOPY, etc.)
- darwin_opt.py    # macOS-specific optimizations
- windows_opt.py   # Windows-specific optimizations
```

## Phase 2: Stream Refactoring (Week 3-4)

### 2.1 Sensory Stream Refactoring
**Current**: Basic PULL socket with JSON/dict handling
**Target**: Zero-copy neural data ingestion

Changes:
- Replace dict-based processing with fixed headers
- Implement ring buffer for incoming neural data
- Add direct FCL injection path
- Remove any video/image processing code
- Add per-cortical-area routing

### 2.2 Motor Stream Refactoring
**Current**: Basic PUB socket with generic publishing
**Target**: Real-time neural command broadcasting

Changes:
- Pre-allocated command buffers
- Fixed-size neural command format
- Lock-free publishing
- Sub-millisecond latency optimization

### 2.3 Visualization Stream Refactoring
**Current**: Generic brain activity publishing
**Target**: Adaptive neural state broadcasting

Changes:
- Implement adaptive quality based on subscribers
- Add compression for large neural states
- Topic-based filtering by detail level
- Zero-copy neural state encoding

### 2.4 Control Stream Updates
**Current**: Mixed REST/control protocol
**Target**: Pure REST API v1 for control operations

Changes:
- Ensure clean REST semantics
- Keep JSON for control (low-frequency)
- Clear separation from data streams

## Phase 3: Protocol Implementation (Week 5-6)

### 3.1 Neural Protocol Handlers
```
Files to create:
- protocols/neuron_flat.py    # Dense neural arrays
- protocols/neuron_sparse.py  # Compressed neural data
- protocols/neuron_multi.py   # Multi-modal neural data
- protocols/cortical_map.py   # Topology updates
```

### 3.2 Protocol Registry
```
Files to create:
- protocol_registry.py  # Dynamic protocol discovery
- protocol_base.py      # Base protocol interface
```

## Phase 4: Flow Control & QoS (Week 7)

### 4.1 Flow Control
```
Files to create:
- flow/neural_flow_controller.py  # Neural-specific flow control
- flow/rate_limiter.py           # Token bucket implementation
- flow/backpressure.py           # Backpressure handling
```

### 4.2 Quality of Service
```
Files to create:
- qos/adaptive_qos.py     # Adaptive quality manager
- qos/priority_scheduler.py # Priority-based scheduling
```

## Phase 5: Testing & Validation (Week 8)

### 5.1 Performance Tests
```
Location: tests/performance/zmq/
- test_zero_copy.py
- test_throughput.py
- test_latency.py
- test_memory_usage.py
```

### 5.2 Integration Tests
```
Location: tests/integration/zmq/
- test_neural_data_flow.py
- test_stream_integration.py
- test_platform_specific.py
```

## Implementation Order

### Week 1-2: Core Infrastructure
1. Create neural data structures and headers
2. Implement ring buffer
3. Create static buffer pools
4. Add platform optimization framework

### Week 3-4: Stream Refactoring
1. Refactor sensory stream for neural data
2. Update motor stream for zero-copy
3. Enhance visualization with adaptive quality
4. Clean up control stream

### Week 5-6: Protocols
1. Implement NEURON_FLAT protocol
2. Add sparse and multi-modal support
3. Create protocol registry
4. Test protocol switching

### Week 7: Flow Control
1. Implement neural flow controller
2. Add rate limiting
3. Create QoS manager
4. Test under load

### Week 8: Testing
1. Performance benchmarks
2. Integration tests
3. Platform-specific validation
4. Documentation updates

## Success Metrics

### Performance Targets
- Neural data latency: <1ms (from current 10-15ms)
- Throughput: 100M neurons/sec (from 1M)
- Memory allocation: 0 in critical path
- CPU usage: <5% (from 15-20%)

### Code Quality
- 100% test coverage for critical paths
- Zero dynamic allocation in data paths
- All structures Rust-FFI compatible
- Platform abstractions complete

## Migration Risks & Mitigation

### Risk 1: Breaking Existing Clients
**Mitigation**: Maintain backward compatibility layer during transition

### Risk 2: Performance Regression
**Mitigation**: Continuous benchmarking, feature flags for rollback

### Risk 3: Platform-Specific Issues
**Mitigation**: Extensive testing on all platforms, gradual rollout

## Dependencies

### External Libraries
- numpy: For efficient array operations
- pyzmq: ZeroMQ Python bindings
- mmap: For memory-mapped buffers
- multiprocessing: For atomic operations

### Internal Dependencies
- feagi_bytes: For protocol encoding/decoding
- CoreAPIService: For FCL injection
- Configuration system: For runtime config

## Next Steps

1. Review and approve this plan
2. Set up performance benchmarking infrastructure
3. Create feature branch for refactoring
4. Begin Phase 1 implementation
5. Weekly progress reviews

## Notes

- All new code must follow RTOS/Rust compatibility guidelines
- No exceptions in critical paths - use error codes
- Fixed-size structures throughout
- Document all platform-specific optimizations
- Maintain compatibility with existing FEAGI_Connector protocol 