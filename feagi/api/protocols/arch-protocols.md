# FEAGI Binary Protocol Architecture

## Overview

This document provides an overview of the FEAGI binary protocol architecture for agent communication. The architecture enables efficient, versioned, binary protocols for communication between FEAGI and external agents.

## Key Components

### 1. Protocol Definitions

Located in `feagi/api/protocols/`:

- **Protocol Base Classes** (`base.py`): Define abstract classes for protocol versioning
- **FCP Implementation** (`fcp.py`): FEAGI Control Protocol for administration and management
- **FVP Implementation** (to be implemented): FEAGI Visualization Protocol for neural visualization
- **FSMP Implementation** (to be implemented): FEAGI Sensorimotor Protocol for sensory/motor data

### 2. Protocol Translator

Located in `feagi/api/protocols/translator.py`:

- Handles translation between binary protocols and internal data structures
- Manages protocol version negotiation
- Preprocesses/postprocesses data as needed for protocols

### 3. API Gateway

Located in `feagi/api/gateway/api_gateway.py`:

- Central entry point for agent communication
- Integrates with protocol translator for message processing
- Manages agent connections and routing
- Handles rate limiting and authentication

## Communication Flow

### Agent Registration Flow

1. Agent connects to FEAGI via REST API and provides supported protocol versions
2. Gateway negotiates compatible protocol versions with ProtocolTranslator
3. Compatible versions are stored with the agent connection
4. ZMQ connections are established for message exchange

### Message Flow: Agent → FEAGI

1. Agent sends binary message via ZMQ
2. Gateway receives and passes to ProtocolTranslator for decoding
3. Translated message is queued for processing 
4. Message is routed to CoreAPIService for handling
5. Response is sent back via the same channels (in reverse)

### Protocol Versioning

The architecture supports versioned protocols with:

1. **Protocol Headers**: Each message includes a protocol ID and version
2. **Version Negotiation**: During agent registration, highest compatible version is selected
3. **Registry System**: Different protocol versions are registered and retrieved as needed

## Binary Message Structure

For each protocol:

1. **Common Header**:
   ```
   +-------------+-------------+
   | Protocol ID | Version     |
   | (1 byte)    | (1 byte)    |
   +-------------+-------------+
   ```

2. **Protocol-Specific Header and Payload**:
   - FCP: Command type, message length, JSON payload
   - FVP: Frame type, timestamp, data length, binary payload
   - FSMP: Channel ID, timestamp, data length, binary payload

## Testing

The architecture includes comprehensive tests in `tests/api/protocols/test_protocols.py` that verify:

1. Protocol registry functionality
2. Message encoding and decoding
3. Version negotiation
4. Protocol translator operation

## Implementation Status

- ✅ Base protocol architecture
- ✅ Protocol registry system
- ✅ Version negotiation
- ✅ Protocol translator implementation
- ✅ FCP implementation
- ❌ FVP implementation (to be completed)
- ❌ FSMP implementation (to be completed)
- ✅ API Gateway integration
- ❌ ZMQ transport integration (to be connected)

## Next Steps

1. Implement FVP and FSMP protocols
2. Integrate ZMQ transport with the gateway for actual message delivery
3. Implement more extensive testing including integration tests
4. Add authentication and security measures
5. Add monitoring and performance metrics for protocol operations 