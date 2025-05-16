# FEAGI Bytes

Binary serialization for FEAGI communication protocols using custom byte structures.

## Overview

This package provides optimized serialization and deserialization for FEAGI communication protocols using custom byte structures. It's designed to be extremely performant for high-throughput neural data transmission.

## Features

- Custom binary serialization optimized for neural data
- Versioned protocol support with capability negotiation
- Support for JSON, images, and various neuron data formats
- Compression support for large payloads
- Compatible with both FEAGI server and clients

## Installation

```bash
pip install feagi_bytes
```

Or install from source:

```bash
git clone https://github.com/feagi/feagi
cd feagi/feagi_bytes
pip install -e .
```

## Quick Usage

```python
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureTranslator

# Create an encoder
encoder = ByteStructureEncoder()

# Encode some JSON data
json_data = {"message": "hello", "value": 42}
encoded_bytes = encoder.encode_json(json_data)

# Create a decoder
decoder = ByteStructureDecoder()

# Decode the data
decoded_data = decoder.decode_json(encoded_bytes)

# Use the translator for higher-level operations
translator = ByteStructureTranslator()
message = translator.create_message({"command": "start", "parameters": {"speed": 10}})
```

## Supported Byte Structures

| Structure ID | Name | Description |
|--------------|------|-------------|
| 1 | JSON | Standard JSON data |
| 8 | RAW_IMAGE | Numpy image arrays |
| 9 | MULTI_HOLDER | Container for multiple structures |
| 16 | NEURON_FLAT | Flat array of neuron data |
| 17 | NEURON_CATEGORIES | Categorized neuron data |

## Documentation

For complete usage documentation, see:

- [Binary Serialization Guide](guide-feagi-bytes.md) - Comprehensive guide to using FEAGI Bytes
- [API Reference](https://feagi.org/docs/api/feagi-bytes) - API reference documentation

## Integration with FEAGI

FEAGI Bytes integrates with the following FEAGI components:

- **API Module**: For encoding/decoding web API responses
- **ZMQ Module**: For efficient binary messaging over ZeroMQ
- **Shared Memory**: For optimized data exchange between processes

## Performance Considerations

For optimal performance:
- Use numpy arrays when possible for large datasets
- Enable compression for large payloads
- Use appropriate structure types for different data (e.g., NEURON_CATEGORIES for multiple cortical areas)

## Development

### Running Tests

```bash
cd feagi_bytes
pytest
```

### Building Documentation

```bash
cd feagi_bytes
mkdocs build
```

## License

Apache License 2.0 