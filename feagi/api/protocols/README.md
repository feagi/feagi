# FEAGI Protocol Architecture

## Custom Byte Structures Implementation

FEAGI uses custom binary byte structures for efficient communication between components. This approach offers significant benefits:

1. **Performance Optimization**: Custom binary formats tailored specifically for neural data
2. **Memory Efficiency**: Direct control over memory layout and minimized overhead
3. **Specialized Structures**: Dedicated formats for different types of data (JSON, images, neuron potentials)
4. **Compatibility**: Simple header-based approach that works across multiple programming languages

## Protocol Structure

All byte structures follow a common pattern:

```
┌─────────────────┬─────────────────┬─────────────────────────┐
│ Structure Type  │ Version Number  │      Message Data      │
│    (1 byte)     │    (1 byte)     │    (variable size)     │
└─────────────────┴─────────────────┴─────────────────────────┘
```

## Versioning System

Each byte structure type has its own versioning, allowing for protocol evolution:

1. **Version Negotiation**: Clients and servers exchange supported versions during handshake
2. **Backward Compatibility**: Servers adapt to client capabilities
3. **Version Registry**: Centralized tracking of supported versions in `SUPPORTED_VERSIONS`
4. **Version-Specific Implementation**: Each version has dedicated encoder/decoder methods

For information on adding new versions, see [Protocol Version Migration Guide](../../docs/protocol_version_migration.md).

## Supported Structure Types

1. **JSON (ID: 1)**
   - For non-performance-critical operations and complex structures
   - Includes protocol and administrative messages

2. **Raw Image (ID: 8)**
   - For efficient transmission of visual data
   - Fixed BGR byte order for compatibility with various systems

3. **Multi Structure Holder (ID: 9)**
   - Container for packaging multiple byte structures together
   - Includes efficient indexing for direct access to specific structures

4. **Neuron Potential - Flat Format (ID: 10)**
   - Optimized for single cortical area neuron data
   - Fast, sequential access to neuron coordinates and potentials

5. **Neuron Potential - Categorized Format (ID: 11)**
   - Optimized for multi-area neuron data
   - Indexed by cortical area for selective processing

## Protocol Types

FEAGI implements three main protocol types over the byte structure format:

1. **FEAGI Control Protocol (FCP)**
   - Administrative and management commands
   - Registration, heartbeat, configuration messages

2. **FEAGI Visualization Protocol (FVP)**
   - Neural activity visualization data
   - Brain structure information

3. **FEAGI Sensorimotor Protocol (FSMP)**
   - Sensory input and motor output data
   - Real-time neural interface

## Directory Structure

- `constants.py`: Protocol IDs, structure IDs, and other constants
- `translator.py`: High-level interface for protocol encoding/decoding
- `byte_structures/`: Core implementation of byte structure format
  - `encoder.py`: Encodes data into byte structures
  - `decoder.py`: Decodes byte structures back to data
  - `utils.py`: Utility functions for working with byte structures

## Usage Example

```python
from feagi.api.protocols import default_translator

# Create a message
message = default_translator.create_handshake_hello(
    agent_id="agent_123",
    agent_type="monitor"
)

# Send message over ZeroMQ or other transport

# On receiving side
decoded = default_translator.decode_message(received_data)
```

## Compression

All byte structures can be optionally compressed using the Deflate algorithm to reduce bandwidth usage:

```python
# Compress before sending
compressed = default_translator.compress_message(message)

# Send compressed message...

# Automatic decompression on receive
decoded = default_translator.decode_message(compressed_message)
``` 