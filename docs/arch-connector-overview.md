# FEAGI Connector Architecture Overview

This document provides an overview of the FEAGI Connector architecture, explaining its design principles, components, and interactions.

## Design Goals

FEAGI Connector is designed with the following goals in mind:

1. **Simplicity**: Provide a simple, high-level API for agent developers
2. **Robustness**: Handle connection issues and protocol complexities
3. **Async-first**: Use modern async/await patterns for efficient I/O
4. **Rust-compatible**: Design with future Rust implementation in mind
5. **Performance**: Minimize overhead for real-time applications

## Architecture Overview

The FEAGI Connector is structured in layers:

1. **High-level Client API**: `FeagiClient` class for agent developers
2. **Protocol-specific Clients**: Specialized clients for each protocol
3. **Transport Layer**: ZeroMQ implementation of the transport

```
┌───────────────────────────────────────┐
│           Agent Application           │
└───────────────────┬───────────────────┘
                    │
┌───────────────────▼───────────────────┐
│            FeagiClient API            │
└───┬───────────────┬───────────────┬───┘
    │               │               │
┌───▼────┐     ┌────▼───┐     ┌────▼────┐
│Command │     │Sensory │     │  Viz    │
│ Client │     │ Client │     │ Client  │
└───┬────┘     └────┬───┘     └────┬────┘
    │               │               │
┌───▼───────────────▼───────────────▼───┐
│           ZeroMQ Transport             │
└───────────────────┬───────────────────┘
                    │
┌───────────────────▼───────────────────┐
│               FEAGI                    │
└───────────────────────────────────────┘
```

## Component Details

### FeagiClient

The main client interface that agent developers interact with. It provides:

- Connection management with FEAGI
- Agent registration
- Sensory data transmission
- Motor data reception
- Visualization data access

### Protocol-specific Clients

#### FeagiCommandClient

Handles the REQ/REP pattern for command-based API:

- Ping/health checks
- Status queries
- Configuration management
- Simulation control

#### FeagiSensoryClient

Handles the DEALER/ROUTER pattern for sensorimotor data:

- Agent registration
- Sensory data transmission
- Motor data reception
- Heartbeat management

#### FeagiVizClient

Handles the DEALER/ROUTER pattern for visualization data:

- Neural activity data
- Brain structure data

### Transport Layer

Currently implemented using ZeroMQ with the following patterns:

- **REQ/REP**: For command-based API (port 5555)
- **DEALER/ROUTER**: For sensorimotor data (port 5558)
- **DEALER/ROUTER**: For visualization data (port 5560)

## Data Flow

### Agent Registration

```
Agent                                    FEAGI
  │                                        │
  ├─── Connect to Command API ────────────►│
  │                                        │
  ├─── Connect to Sensory API ────────────►│
  │                                        │
  ├─── Send "hello" message ──────────────►│
  │                                        │
  │◄─── Receive "welcome" message ─────────┤
  │                                        │
  ├─── Start heartbeat loop ──────────────►│
  │                                        │
```

### Sensory Data Transmission

```
Agent                                    FEAGI
  │                                        │
  ├─── Encode sensory data ────────────────┤
  │                                        │
  ├─── Send data to cortical area ────────►│
  │                                        │
  │                                        ├─── Process in brain
  │                                        │
```

### Motor Data Reception

```
Agent                                    FEAGI
  │                                        │
  │◄─── Receive motor data ────────────────┤
  │                                        │
  ├─── Process with callback ──────────────┤
  │                                        │
```

## Future Directions

1. **Rust Implementation**: The architecture is designed to be implementable in Rust
2. **WebSocket Transport**: Add alternative transport for web-based agents
3. **Authentication**: Enhance security with token-based authentication
4. **Compression**: Add optional compression for large data transfers
5. **Protocol Versioning**: Support multiple protocol versions 