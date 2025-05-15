# FEAGI Protocol Architecture

## Protocol Transition Notice

**IMPORTANT**: FEAGI is in the process of transitioning from custom binary protocols and Protocol Buffers to **Cap'n Proto** for all message serialization. This transition offers significant performance improvements including zero-copy deserialization, faster serialization, and improved compatibility.

## Current Architecture

The protocol architecture consists of:

1. **Cap'n Proto Implementation** (Current Focus)
   - `translator.py`: Main interface for Cap'n Proto schema loading and message creation
   - Schemas defined in the `feagi_capnp` directory

2. **Legacy Protocol Implementation** (Deprecated)
   - `binary.py`: Contains binary serialization/deserialization utilities
   - Protocol version implementations in subdirectories (`fcp/`, `fsmp/`, `fvp/`)
   - Root protocol files (`fcp.py`, `fsmp.py`, `fvp.py`)

3. **Protocol Base Definitions** (Maintained)
   - `base.py`: Defines protocol IDs, versioning, and registry

## Protocol Types

FEAGI implements three main protocol types:

1. **FEAGI Control Protocol (FCP)**
   - Administrative and management commands
   - Agent registration, configuration, status updates

2. **FEAGI Sensorimotor Protocol (FSMP)**
   - Sensory input and motor output data
   - High-performance streaming of arrays

3. **FEAGI Visualization Protocol (FVP)**
   - Neural activity data for visualization
   - Brain state monitoring

## ZeroMQ Implementation

The protocols are implemented using ZeroMQ with the ROUTER-DEALER pattern for efficient client-server communication. See `feagi/api/zmq` for implementation details.

## Migration Guide

When working with the protocol system:

1. For new code, use the Cap'n Proto implementation via the `translator.py` module
2. Existing code using legacy protocols will continue to work but is deprecated
3. Test both implementations during the transition phase

## Future Work

- Complete removal of Protocol Buffers dependencies
- Full refactor of protocol modules to use Cap'n Proto exclusively
- Comprehensive test coverage for all protocol operations 