# FEAGI Communication Protocols (Cap'n Proto)

This directory contains Cap'n Proto schema definitions for all FEAGI communication protocols.

## Directory Structure

```
feagi_capnp/
├── common/                     # Shared definitions across all protocols
│   └── constants.capnp         # Common constants and types
├── handshake/                  # Connection establishment protocol
│   └── v1/                     # Version 1
│       └── handshake.capnp     # Handshake protocol definition
├── fcp/                        # FEAGI Control Protocol
│   └── v1/                     # Version 1
│       └── fcp.capnp           # FCP definitions
├── fsmp/                       # FEAGI Sensorimotor Protocol
│   └── v1/                     # Version 1
│       └── fsmp.capnp          # FSMP definitions
└── fvp/                        # FEAGI Visualization Protocol
    └── v1/                     # Version 1
        └── fvp.capnp           # FVP definitions
```

## Why Cap'n Proto?

FEAGI previously used Protocol Buffers for communication but has switched to Cap'n Proto for several key advantages:

1. **Zero-Copy Deserialization**: Cap'n Proto uses a binary format that doesn't require deserialization, which significantly improves performance.
2. **Faster Serialization**: Cap'n Proto is consistently faster than Protocol Buffers, especially for large data structures.
3. **Memory Efficiency**: The memory representation is the same as the serialized format, reducing memory overhead.
4. **Forward/Backward Compatibility**: Cap'n Proto has robust schema evolution support, allowing for seamless upgrades.
5. **Built-in RPC Capabilities**: While not currently used by FEAGI, Cap'n Proto includes a powerful RPC system.

These benefits make Cap'n Proto particularly well-suited for FEAGI's high-performance sensorimotor and visualization data streams.

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

## Using the Cap'n Proto Schemas

### Python

Python clients can use the pycapnp library:

```bash
pip install pycapnp
```

Example usage:

```python
import os
import capnp
from capnp import KjException
import time

# Load schemas from feagi_capnp
schema_path = "/path/to/feagi_capnp"
constants_schema = capnp.load(os.path.join(schema_path, "common/constants.capnp"))
handshake_schema = capnp.load(os.path.join(schema_path, "handshake/v1/handshake.capnp"))

# Create a message
hello = handshake_schema.HelloMessage.new_message(
    agentId="test-agent",
    agentType="test-client"
)

# Timestamp
timestamp = constants_schema.Timestamp.new_message(timeMs=int(time.time() * 1000))
hello.timestamp = timestamp

# Serialize
data = hello.to_bytes()

# Deserialize
parsed_hello = handshake_schema.HelloMessage.from_bytes(data)
```

You can also import the feagi_capnp package directly:

```python
import feagi_capnp
from feagi_capnp import ProtocolID, HelloMessage, HandshakeMessageType

# Use the exported types
hello = HelloMessage.new_message(
    agentId="test-agent",
    agentType="test-client"
)
```

### JavaScript

For JavaScript, use the capnp-ts library:

```bash
npm install capnp-ts
```

### Rust

For Rust, use the capnproto-rust crate:

```toml
[dependencies]
capnp = "0.19.0"
```

Add a build.rs file:

```rust
fn main() {
    capnpc::CompilerCommand::new()
        .src_prefix("feagi_capnp")
        .file("feagi_capnp/common/constants.capnp")
        .file("feagi_capnp/handshake/v1/handshake.capnp")
        .file("feagi_capnp/fcp/v1/fcp.capnp")
        .file("feagi_capnp/fsmp/v1/fsmp.capnp")
        .file("feagi_capnp/fvp/v1/fvp.capnp")
        .run().expect("compiling schema");
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

## Implementation Notes

When implementing a new client for FEAGI:

1. Use ZeroMQ (ZMQ) as the transport layer
2. Implement full handshake protocol for connection establishment
3. After handshake, use optimized message formats for data exchange
4. Use topic-based routing for protocol identification 