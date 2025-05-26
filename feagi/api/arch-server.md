# FEAGI Server Architecture

This document explains the architecture of the FEAGI server components and how they work together to implement the process architecture described in `docs/feagi_processes.md`.

## Overview

FEAGI employs a process-based architecture designed for high performance and efficient resource utilization. The architecture is built on three principles:

1. **Process Separation**: Critical computation is separated from non-critical processes to ensure consistent neural simulation performance.
2. **Priority-based Resource Allocation**: System resources are allocated based on process priority.
3. **Independent Process Management**: Each major system component operates as an independent process with clear responsibilities.

## Process Types and Priorities

FEAGI processes are categorized into three priority levels:

### Priority 1 (Critical - Real-time)
These processes handle the core neural simulation and must maintain real-time performance:

1. **Burst Engine**: Manages neuron firing dynamics, threshold detection, and refractory periods.
2. **Connectome Manager**: Handles access to neuron and synapse data structures.
3. **FCL Manager**: Maintains the Fire Candidate List and provides efficient queries.
4. **Memory & Learning Manager**: Applies plasticity rules to synaptic weights.

### Priority 2 (Important - Near Real-time)
These processes handle important but less time-critical operations:

1. **FQ Sampler**: Periodically extracts data from the FCL for visualization and motor output.
2. **PNS Message Broker (ZMQ Server)**: Manages communication with peripherals and external systems.
3. **Resource Manager**: Monitors and allocates system resources based on process demands.

### Priority 3 (Background - Best Effort)
These processes handle optional or background operations:

1. **Web Server (REST API)**: Provides API endpoints for monitoring and control.
2. **Stem Cell Manager**: Handles neurogenesis and synaptogenesis.
3. **Sleep Manager**: Manages memory consolidation during inactive periods.

## Key Components

### 1. Process Manager (`feagi/process_manager.py`)

The Process Manager is responsible for:
- Starting processes in priority order
- Monitoring process health
- Allocating resources based on priority
- Managing graceful shutdown

It ensures that Priority 1 processes are fully initialized before starting Priority 2 processes, and so on.

### 2. Main Entry Point (`feagi/main.py`)

This is the primary entry point for running the complete FEAGI system. It:
- Parses command-line arguments
- Checks dependencies
- Initializes the Process Manager
- Sets up signal handlers for graceful shutdown

### 3. Core API (`feagi/core/__init__.py`)

This module provides the critical (Priority 1) components:
- Creates and initializes the core FEAGI instance
- Initializes the Burst Engine, Connectome Manager, FCL Manager, and Memory Manager
- Provides a uniform API for accessing these components

### 4. ZMQ Server (`feagi/api/zmq/server.py`)

The ZMQ server implements the PNS Message Broker (Priority 2) and:
- Provides multiple communication patterns (REQ/REP, PUB/SUB, PUSH/PULL)
- Handles real-time streaming of neural activity
- Manages connections with external peripherals

### 5. REST API (`feagi/api/rest/app.py`)

The REST API server (Priority 3) provides:
- HTTP endpoints for monitoring and control
- Swagger UI documentation
- WebSocket streaming for visualization

### 6. API Gateway (`feagi/api/gateway.py`)

The API Gateway connects all API interfaces to the core components:
- Provides a uniform API for both REST and ZMQ interfaces
- Manages connection to local or remote core components
- Handles authentication and authorization

## Startup Sequence

When you run `python -m feagi.main`, the following sequence occurs:

1. Dependencies are checked
2. The Process Manager is initialized
3. Critical (Priority 1) processes are started:
   - Core API is initialized
   - Burst Engine, Connectome Manager, FCL Manager, and Memory Manager are initialized
4. Important (Priority 2) processes are started:
   - ZMQ server is initialized and started
5. Background (Priority 3) processes are started:
   - REST API server is started in a separate process

## Standalone Components

For development and testing purposes, you can run individual components:

### REST API Only

```
python -m feagi.api.server --host 127.0.0.1 --port 8000
```

### ZMQ Server Only

```
python -m feagi.api.zmq.main --host 127.0.0.1
```

## Common Issues and Troubleshooting

### Port Conflicts

If you encounter port conflicts (e.g., "Address already in use"), you can:
1. Kill any running FEAGI processes: `pkill -f "python -m feagi.main"`
2. Specify different ports: `python -m feagi.main --api-port 8001 --zmq-req-port 5565`

### ZMQ Server Asyncio Issues

The ZMQ server uses asyncio and may show warnings like "coroutine was never awaited". These are expected as the server runs in a background thread.

If you see "Task got Future attached to a different loop" errors, this indicates an issue with the event loop management. In this case:
1. Kill all FEAGI processes
2. Restart with `python -m feagi.main`

### REST API Errors

If the REST API shows "NoneType has no attribute" errors, this usually means the core API is not properly initialized. This typically happens when the REST API is started before the core components are ready.

To fix this, always use `python -m feagi.main` to start the complete system, which ensures components are initialized in the correct order.

## Architecture Diagram

```
┌────────────────────────────────────────────────────────────────┐
│                      FEAGI Main Process                         │
│                                                                │
│  ┌─────────────────┐  ┌────────────────┐  ┌─────────────────┐  │
│  │   Priority 1    │  │   Priority 2   │  │   Priority 3    │  │
│  │                 │  │                │  │                 │  │
│  │  Core API       │  │  ZMQ Server    │  │  REST API       │  │
│  │  - Burst Engine │  │  - REQ/REP     │  │  - HTTP Server  │  │
│  │  - Connectome   │  │  - PUB/SUB     │  │  - WebSockets   │  │
│  │  - FCL Manager  │  │  - PUSH/PULL   │  │  - Swagger UI   │  │
│  │  - Memory Mgr   │  │  - Stream      │  │                 │  │
│  └─────────────────┘  └────────────────┘  └─────────────────┘  │
│                                                                │
│                    Process Manager                             │
└────────────────────────────────────────────────────────────────┘
                              │
                              │
                              ▼
┌────────────────────────────────────────────────────────────────┐
│                       External Clients                          │
│                                                                │
│  ┌─────────────────┐  ┌────────────────┐  ┌─────────────────┐  │
│  │   Web Browser   │  │    ZMQ Client  │  │   Peripherals   │  │
│  │   - Dashboard   │  │    - Sensors   │  │   - Robots      │  │
│  │   - Monitoring  │  │    - Actuators │  │   - Cameras     │  │
│  └─────────────────┘  └────────────────┘  └─────────────────┘  │
└────────────────────────────────────────────────────────────────┘
``` 