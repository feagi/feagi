# Core Module Architecture

This document describes the architectural design and organization of FEAGI's Core module.

## Architectural Overview

The Core module serves as the backbone of FEAGI, providing critical infrastructure components that enable other modules to function. It is designed with several key principles:

1. **Resource Awareness**: Optimize performance based on available hardware
2. **State Management**: Provide reliable tracking of system state
3. **Process Orchestration**: Coordinate multiple processes efficiently
4. **Backend Abstraction**: Support multiple computational backends

## Component Architecture

### State Manager

![State Manager Architecture](../../docs/assets/state-manager-diagram.png)

The State Manager is built on a memory-mapped state design for high performance:

```
┌───────────────────────┐
│  Memory-Mapped State  │
├───────────────────────┤
│ - Genome State        │
│ - Connectome State    │
│ - Service States      │
│ - Simulation Control  │
└─────────┬─────────────┘
          │
          ▼
┌───────────────────────┐
│  State Manager API    │
├───────────────────────┤
│ - Getters/Setters     │
│ - Observers           │
│ - Transactions        │
└─────────┬─────────────┘
          │
          ▼
┌───────────────────────┐
│  Consumer Components  │
└───────────────────────┘
```

#### Key Design Elements:

1. **Memory-Mapped State**: Uses a shared memory region to store state, allowing near-zero overhead access
2. **Observer Pattern**: Components register as observers to receive state change notifications
3. **Atomic Transactions**: Supports atomic operations through the transaction system
4. **Thread Safety**: All operations are thread-safe, using appropriate locking mechanisms

### Resource Manager

The Resource Manager employs a hierarchical design for effective resource allocation:

```
┌───────────────────────┐
│  Resource Detection   │
├───────────────────────┤
│ - CPU Cores           │
│ - GPU Capabilities    │
│ - Memory              │
└─────────┬─────────────┘
          │
          ▼
┌───────────────────────┐
│ Resource Allocation   │
├───────────────────────┤
│ - Process Resources   │
│ - Thread Resources    │
└─────────┬─────────────┘
          │
          ▼
┌───────────────────────┐
│   Process Lifecycle   │
├───────────────────────┤
│ - Start/Terminate     │
│ - Monitoring          │
│ - Heartbeats          │
└───────────────────────┘
```

#### Key Design Elements:

1. **Resource Detection**: Automatically detects available CPU cores, GPUs, and memory
2. **Process Registry**: Maintains a registry of running processes with their resource allocations
3. **Monitoring System**: Periodically checks process health through heartbeats
4. **Resource Reclamation**: Reclaims resources when processes terminate

### Backend Framework

The Backend Framework uses an abstract factory pattern to provide unified access to computational resources:

```
┌───────────────────────┐
│  Backend Interface    │
├───────────────────────┤
│ - Common API          │
│ - Capability Query    │
└─────────┬─────────────┘
          │
          ▼
┌─────────┴─────────────┐
│                       │
▼                       ▼
┌───────────────┐ ┌───────────────┐
│  CPU Backend  │ │ WebGPU Backend│
└───────────────┘ └───────────────┘
```

#### Key Design Elements:

1. **Common Interface**: All backends implement the same interface for interchangeability
2. **Capability-Based Selection**: Backends are selected based on available hardware
3. **Optimized Implementations**: Each backend optimizes for its specific hardware
4. **Fallback Mechanism**: Automatically falls back to CPU if requested backend is unavailable

### Genome Transaction System

The Genome Transaction System ensures consistent changes to the genome and connectome:

```
┌───────────────────────┐
│  Transaction Context  │
├───────────────────────┤
│ - Change Recording    │
│ - Validation          │
└─────────┬─────────────┘
          │
          ▼
┌───────────────────────┐
│  Commit Processing    │
├───────────────────────┤
│ - Atomic Application  │
│ - Rollback            │
└─────────┬─────────────┘
          │
          ▼
┌───────────────────────┐
│ Observer Notification │
└───────────────────────┘
```

#### Key Design Elements:

1. **Two-Phase Commit**: Changes are recorded first, then applied atomically
2. **Rollback Support**: Failed transactions can be rolled back
3. **Observer Notifications**: Observers are notified of successful changes
4. **State Tracking**: Transaction state is tracked in the State Manager

### Security Components

The Security components provide authentication and encryption:

```
┌───────────────────────┐
│  Authentication       │
├───────────────────────┤
│ - Token Validation    │
│ - Permission Control  │
└───────────────────────┘

┌───────────────────────┐
│  Encryption           │
├───────────────────────┤
│ - ZMQ Encryption      │
│ - Secure Storage      │
└───────────────────────┘
```

## Interaction Patterns

### Process Initialization Flow

```
1. ResourceManager detects available hardware
2. ResourceManager allocates resources for core processes
3. Processes are started with allocated resources
4. StateManager tracks initialization progress
5. Components register as observers for state changes
6. System becomes ready when all components report ready state
```

### Backend Selection Flow

```
1. Application requests a backend (or best available)
2. BackendInterface checks available hardware
3. BackendInterface instantiates appropriate backend
4. Application interacts with backend through common interface
5. Computation is optimized for chosen backend
```

### Genome Transaction Flow

```
1. Component creates a transaction
2. Transaction records intended changes
3. Component commits transaction
4. StateManager updates to SYNCING state
5. Changes are applied atomically to genome
6. Connectome is updated to reflect changes
7. StateManager updates to SYNC_COMPLETE state
8. Observers are notified
```

## Performance Considerations

### State Manager

- **Memory-Mapped Design**: Provides near-zero overhead for state access
- **Minimal Locking**: Uses fine-grained locks only where necessary
- **Efficient Observer Pattern**: Observers are weakly referenced to prevent memory leaks

### Resource Manager

- **Dynamic Resource Allocation**: Adjusts resource allocation based on system load
- **Periodic Monitoring**: Monitors process health at configurable intervals
- **Resource Reclamation**: Automatically reclaims resources from terminated processes

### Backend Framework

- **Lazy Initialization**: Backends are initialized only when needed
- **Capability Caching**: Hardware capabilities are detected once and cached
- **Memory Transfer Optimization**: Minimizes data transfer between CPU and GPU

## Error Handling

### Failure Modes

| Component | Failure Mode | Recovery Strategy |
|-----------|--------------|-------------------|
| State Manager | Memory mapping failure | Fall back to in-memory state |
| State Manager | Lock contention | Exponential backoff retry |
| Resource Manager | Process crash | Detect termination, reclaim resources |
| Resource Manager | Resource exhaustion | Deny allocation, log warning |
| Backend | GPU initialization failure | Fall back to CPU backend |
| Transaction | Commit failure | Roll back changes, notify observers |

### Logging Strategy

The Core module uses structured logging with severity levels:

1. **DEBUG**: Detailed information for debugging
2. **INFO**: Normal operations and state changes
3. **WARNING**: Non-critical issues that might require attention
4. **ERROR**: Critical issues that prevent normal operation
5. **CRITICAL**: System-wide failures that require immediate attention

### Structured Output

Log entries include:
- Timestamp
- Severity level
- Component identifier
- State information (where appropriate)
- Message with contextual information

## Future Directions

1. **Cross-Machine Resource Management**: Extend resource management across multiple machines
2. **Additional Backend Support**: Add support for specialized hardware like NPUs or TPUs
3. **Fine-Grained Permission System**: Enhance security with role-based access control
4. **Dynamic Backend Switching**: Allow switching backends at runtime based on workload
