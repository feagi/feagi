# FEAGI Inter-Process Communication Architecture

*Last Updated: May 15, 2025*

## Overview

This document describes the Inter-Process Communication (IPC) architecture used in FEAGI to enable efficient communication between the API server and the burst engine.

FEAGI uses a high-performance shared memory approach for IPC rather than socket-based communication. This architecture provides several benefits:

1. **Lower Latency**: Direct memory access eliminates network stack traversal
2. **Higher Throughput**: No serialization/deserialization overhead
3. **Reduced CPU Usage**: No constant polling or socket I/O operations
4. **Better Stability**: Fewer components that can fail
5. **Simpler Codebase**: More direct interaction between components

## Architecture Components

The IPC architecture consists of the following components:

### 1. Shared Memory Manager

The Shared Memory Manager handles the creation, mapping, and synchronization of shared memory regions:

- Creates memory-mapped files for large data structures
- Handles locking to prevent concurrent access
- Provides clean API for both Python and future Rust implementations

### 2. Event Communication System

The Event Communication System provides a way for processes to notify each other about state changes:

- Uses lightweight file-based semaphores for signaling
- Implements an event queue for non-blocking operation
- Provides synchronous and asynchronous notification modes

### 3. Process Manager

The Process Manager coordinates the startup, monitoring, and shutdown of processes:

- Starts processes in priority order
- Sets process CPU affinity and priority
- Monitors process health
- Handles graceful shutdown

### 4. Data Exchange Format

The Data Exchange Format defines how structured data is shared between processes:

- Uses memory-mapped NumPy arrays for numerical data
- Implements shared dictionaries for metadata
- Defines binary formats optimized for cross-language compatibility

## Implementation Details

### Shared Memory Data Structures

FEAGI uses the following shared memory data structures:

1. **Neuron Arrays**: Memory-mapped NumPy arrays storing neuron state
2. **Synapse Arrays**: Memory-mapped NumPy arrays storing synaptic connections
3. **Fire Candidate List (FCL)**: Shared memory queue of recently fired neurons
4. **Configuration Dictionary**: Shared dictionary for runtime configuration

### Synchronization Mechanisms

To ensure data consistency across processes, FEAGI employs:

1. **File Locks**: For exclusive access to shared memory regions
2. **Memory Barriers**: To ensure visibility of changes across CPU cores
3. **Atomic Operations**: For lock-free updates where possible
4. **Versioned Access**: To detect and handle concurrent modifications

### Event Notification

The event notification system works as follows:

1. When the burst engine updates shared data, it sets an event flag
2. The API service checks event flags when handling requests
3. For long-polling requests, the API service can wait for specific events
4. Events are categorized by priority to prevent flooding

### Error Handling

The IPC architecture includes robust error handling:

1. **Process Monitoring**: Detects crashed processes and restarts them
2. **Deadlock Detection**: Detects and resolves synchronization deadlocks
3. **Timeout Mechanisms**: Prevents indefinite blocking on IPC operations
4. **Graceful Degradation**: Falls back to cached data when shared memory is unavailable

## Performance Characteristics

The shared memory IPC architecture provides significant performance improvements:

1. **Latency**: < 100 microseconds for typical IPC operations (vs. 1-10ms for socket-based approaches)
2. **Throughput**: Limited only by memory bandwidth (10+ GB/s on modern systems)
3. **CPU Overhead**: < 5% CPU usage for IPC operations
4. **Memory Efficiency**: Shared data structures eliminate redundant copies

## Rust Migration Considerations

The IPC architecture is designed to facilitate future migration to Rust:

1. **FFI-Compatible Types**: All shared data structures use types that can be accessed from both Python and Rust
2. **Minimal Dependencies**: The implementation uses standard libraries to simplify FFI integration
3. **Clean Interfaces**: Well-defined component interfaces simplify incremental migration
4. **Memory Safety**: The architecture enforces access patterns that prevent memory corruption

## Implementation Status

The shared memory IPC architecture is being implemented in phases:

1. **Phase 1**: Basic shared memory manager and neuron arrays (Completed)
2. **Phase 2**: Event notification system and process manager (In Progress)
3. **Phase 3**: Full shared memory FCL and synapse arrays (Planned)
4. **Phase 4**: Advanced synchronization and error handling (Planned)
5. **Phase 5**: Rust integration and optimizations (Future)

## Related Documentation

- [System Overview](arch-system-overview.md)
- [GPU Architecture](arch-gpu.md)
- [State Management](arch-state-management.md)
