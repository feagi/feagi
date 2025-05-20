# FEAGI Bytes: Binary Serialization Guide

FEAGI Bytes is a high-performance binary serialization library designed specifically for FEAGI's communication protocols. It provides optimized encoding and decoding for neural data using custom byte structures.

## Key Components

### ByteStructureEncoder

The encoder provides methods to convert various data types into binary format:

```python
from feagi_bytes import ByteStructureEncoder

encoder = ByteStructureEncoder()

# Encode JSON data
json_bytes = encoder.encode_json({"command": "start", "value": 42})

# Encode image data
image_bytes = encoder.encode_raw_image(numpy_image_array)

# Encode neuron data
neuron_bytes = encoder.encode_neuron_flat(
    cortical_ids=["visual_cortex", "auditory_cortex"],
    x_coords=[1, 2, 3],
    y_coords=[4, 5, 6],
    z_coords=[7, 8, 9],
    potentials=[0.5, 0.7, 0.9]
)

# Compress data for more efficient transmission
compressed_data = encoder.compress(large_data_bytes)
```

### ByteStructureDecoder

The decoder converts binary data back to Python objects:

```python
from feagi_bytes import ByteStructureDecoder

decoder = ByteStructureDecoder()

# Decode JSON data
json_data = decoder.decode_json(json_bytes)

# Decode image data
image_array = decoder.decode_raw_image(image_bytes)

# Decode neuron data
neuron_data = decoder.decode_neuron_flat(neuron_bytes)

# Handle automatic decoding based on structure type
decoded_data = decoder.decode(some_bytes)

# Decompress data
original_data = decoder.decompress(compressed_data)
```

### ByteStructureTranslator

The translator provides higher-level protocol operations:

```python
from feagi_bytes import ByteStructureTranslator

translator = ByteStructureTranslator()

# Create protocol messages
message_bytes = translator.create_message({"command": "register", "parameters": {"name": "agent1"}})

# Decode incoming messages
received_message = translator.decode_message(received_bytes)

# Handle version negotiation with clients
translator.register_client_capabilities("client1", {"structure_versions": {"1": [1, 2]}})
version = translator.get_supported_version("client1", 1)  # Get compatible JSON version

# Create specialized neuron data messages
neuron_message = translator.create_neuron_data_message({
    "visual_cortex": {
        "x": [1, 2, 3],
        "y": [4, 5, 6],
        "z": [7, 8, 9],
        "potentials": [0.5, 0.6, 0.7]
    }
})
```

## Supported Data Types

FEAGI Bytes can encode and decode the following data types:

| Structure ID | Name | Description |
|--------------|------|-------------|
| 1 | JSON | Standard JSON data |
| 8 | RAW_IMAGE | Numpy image arrays |
| 9 | MULTI_HOLDER | Container for multiple structures |
| 16 | NEURON_FLAT | Flat array of neuron data |
| 17 | NEURON_CATEGORIES | Categorized neuron data |

## Performance Considerations

### Compression

For large payloads, compression can significantly reduce transmission size:

```python
# Compress before sending
large_data = encoder.encode_json(very_large_dict)
compressed = encoder.compress(large_data)

# Send compressed data...

# Receiver can automatically handle compressed data
decoded = decoder.decode_json(compressed)  # Auto-decompression happens internally
```

### Memory Optimization

When working with large arrays, use native numpy arrays to minimize memory usage:

```python
import numpy as np

# More efficient than regular Python lists
x_coords = np.array([1, 2, 3, ...], dtype=np.int32)
y_coords = np.array([4, 5, 6, ...], dtype=np.int32)
z_coords = np.array([7, 8, 9, ...], dtype=np.int32)
potentials = np.array([0.5, 0.6, 0.7, ...], dtype=np.float32)

# Encode with numpy arrays for better performance
neuron_bytes = encoder.encode_neuron_flat(
    cortical_ids=["visual_cortex"],
    x_coords=x_coords,
    y_coords=y_coords, 
    z_coords=z_coords,
    potentials=potentials
)
```

## Protocol Version Handling

FEAGI Bytes supports versioned protocols for backward compatibility:

```python
# Register client capabilities
translator.register_client_capabilities("client1", {
    "structure_versions": {
        "1": [1],  # JSON: version 1
        "16": [1, 2]  # NEURON_FLAT: versions 1 and 2
    }
})

# Create message with negotiated version
message = translator.create_message(data, client_id="client1")
```

## Integration with FEAGI Core

FEAGI Bytes integrates with core FEAGI components:

1. **API Module**: For encoding/decoding web API responses
2. **ZMQ Module**: For efficient binary messaging over ZeroMQ
3. **Shared Memory**: For optimized data exchange between processes

## Error Handling

Handle potential errors during encoding/decoding:

```python
try:
    decoded_data = decoder.decode_json(received_bytes)
except ValueError as e:
    # Handle malformed data
    print(f"Error decoding JSON data: {e}")
    # Implement appropriate error recovery strategy
```

## Examples

### Basic Message Exchange

```python
# Sender side
from feagi_bytes import ByteStructureTranslator

sender = ByteStructureTranslator()
message = {"type": "command", "action": "start", "parameters": {"speed": 10}}
binary_message = sender.create_message(message)

# Send binary_message through network...

# Receiver side
receiver = ByteStructureTranslator()
decoded = receiver.decode_message(binary_message)
# decoded == {"type": "command", "action": "start", "parameters": {"speed": 10}}
```

### Neural Data Transmission

```python
# Create efficient neuron activity message
neuron_message = translator.create_neuron_data_message({
    "visual_cortex": {
        "x": [1, 2, 3, 4, 5],
        "y": [6, 7, 8, 9, 10],
        "z": [11, 12, 13, 14, 15],
        "potentials": [0.1, 0.2, 0.3, 0.4, 0.5]
    },
    "motor_cortex": {
        "x": [20, 21, 22],
        "y": [23, 24, 25],
        "z": [26, 27, 28],
        "potentials": [0.7, 0.8, 0.9]
    }
})

# This message is optimized for efficient transmission of neural activity
``` 