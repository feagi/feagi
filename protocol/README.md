# FEAGI Communication Protocols

This directory contains Protocol Buffer definitions for all FEAGI communication protocols.

## Directory Structure

```
protocol/
├── common/                     # Shared definitions across all protocols
│   └── constants.proto         # Common constants and types
├── handshake/                  # Connection establishment protocol
│   └── v1/                     # Version 1
│       └── handshake.proto     # Handshake protocol definition
├── fcp/                        # FEAGI Control Protocol
│   └── v1/                     # Version 1
│       └── fcp.proto           # FCP definitions
├── fsmp/                       # FEAGI Sensorimotor Protocol
│   └── v1/                     # Version 1
│       └── fsmp.proto          # FSMP definitions
└── fvp/                        # FEAGI Visualization Protocol
    └── v1/                     # Version 1
        └── fvp.proto           # FVP definitions
```

## Performance Optimization

FEAGI's protocol design uses a two-phase approach for optimal performance:

1. **Handshake Phase**: During connection establishment, full headers and version negotiation occur.
2. **Data Transfer Phase**: Once connected, messages contain minimal headers for maximum throughput.

This significantly reduces overhead, especially for high-frequency sensorimotor and visualization data.

## Protocol Summary

### Handshake Protocol

Used for connection establishment and version negotiation between agents and FEAGI.

### FCP (FEAGI Control Protocol)

Used for agent registration, heartbeats, status requests, and general control operations.

### FSMP (FEAGI Sensorimotor Protocol)

Used for exchanging sensory data (agent to FEAGI) and motor commands (FEAGI to agent).

### FVP (FEAGI Visualization Protocol)

Used for visualization of brain structure and neural activity.

## Generating Client Libraries

### Python

Generate Python code:

```bash
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. protocol/common/constants.proto protocol/handshake/v1/handshake.proto protocol/fcp/v1/fcp.proto protocol/fsmp/v1/fsmp.proto protocol/fvp/v1/fvp.proto
```

### JavaScript

Install the necessary packages:

```bash
npm install protobufjs
```

Generate JavaScript code:

```bash
npx pbjs -t static-module -w commonjs -o feagi_protocol.js protocol/**/*.proto
```

### Rust

Add protobuf dependencies to Cargo.toml:

```toml
[dependencies]
prost = "0.11"
prost-types = "0.11"

[build-dependencies]
prost-build = "0.11"
```

Create a build.rs file:

```rust
fn main() {
    let protos = [
        "protocol/common/constants.proto", 
        "protocol/handshake/v1/handshake.proto",
        "protocol/fcp/v1/fcp.proto",
        "protocol/fsmp/v1/fsmp.proto",
        "protocol/fvp/v1/fvp.proto"
    ];
    
    prost_build::compile_protos(&protos, &["."]).unwrap();
}
```

## Version Policy

Each protocol evolves independently with its own versioning. During the handshake phase, an agent advertises which protocol versions it supports, and the server chooses compatible versions.

| Protocol   | Current Version | Last Updated |
|------------|----------------|--------------|
| Handshake  | 1              | 2024         |
| FCP        | 1              | 2024         |
| FSMP       | 1              | 2024         |
| FVP        | 1              | 2024         |

## Protocol Documentation

### Common Definitions
- **ProtocolID**: Identifies the protocol type (FCP, FSMP, FVP)
- **Timestamp**: Common timestamp format used across all protocols

### Handshake Protocol
- Connection establishment protocol with multiple steps:
  1. **HELLO**: Initial client greeting
  2. **WELCOME**: Server welcome response
  3. **CAPABILITIES**: Client capabilities
  4. **CONFIGURATION**: Server configuration response
  5. **READY/START**: Final handshake

### FCP (FEAGI Control Protocol)
- **Registration**: Register an agent (after handshake)
- **Deregistration**: Deregister an agent
- **Heartbeat**: Keep connection alive
- **Status**: Request/response for FEAGI status
- **Configure**: Configure FEAGI

### FSMP (FEAGI Sensorimotor Protocol)
- **Sensory Data**: Send sensory information to FEAGI
  - Vision
  - Audio
  - Proprioception
  - Tactile
- **Motor Data**: Receive motor commands from FEAGI
  - Movement
  - Speech
  - Manipulation

### FVP (FEAGI Visualization Protocol)
- **Structure Data**: Brain structure information
- **Activity Data**: Neural activity information
- **Configuration**: Visualization settings

## Implementation Notes

When implementing a new client for FEAGI:

1. Use ZeroMQ (ZMQ) as the transport layer
2. Implement full handshake protocol for connection establishment
3. After handshake, use optimized message formats for data exchange
4. Use topic-based routing for protocol identification 

## Testing

An end-to-end test for all protocols is available at `tests/api/protocols/test_end_to_end.py`. This test:

1. Creates a mock server and client
2. Tests the full communication cycle for all protocols:
   - Handshake/Registration
   - FCP control messages
   - FSMP sensory and motor data 
   - FVP structure and activity visualization

## Status

The protocol refactoring is complete. Both client and server implementations have been updated to use the versioned Protocol Buffer definitions.

The old implementation has been removed, and all code now uses the new protocol structure based on:
- Shared common definitions (protocol/common/constants.proto)
- Versioned protocol interfaces (v1 folders for each protocol)
- Protocol Buffer generated client libraries for type safety and performance 