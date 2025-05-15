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

## Usage

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

## License

Apache License 2.0 