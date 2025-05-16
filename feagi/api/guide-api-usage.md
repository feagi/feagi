# API Usage Guide

*Last Updated: May 15, 2025*

This guide explains how to use the FEAGI API for both internal development and external client applications.

## Internal Usage

### Using CoreAPIService

When developing internal FEAGI components, always use the CoreAPIService to access core functionality:

```python
from feagi.api.core.services import CoreAPIService

# Create the service with required dependencies
service = CoreAPIService(connectome_manager=connectome_manager, 
                         state_manager=state_manager)

# Examples of common operations
cortical_areas = service.get_cortical_area_list()
genome_loaded = service.genome_is_loaded()
service.create_cortical_area(properties=area_properties)
```

### Dependency Injection Pattern

When creating FastAPI endpoints, use the dependency injection pattern:

```python
from fastapi import APIRouter, Depends, HTTPException
from feagi.api.core.services import CoreAPIService
from feagi.api.rest.dependencies import get_core_api_service

router = APIRouter()

@router.get("/cortical_areas")
async def get_cortical_areas(
    api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Get a list of all cortical areas."""
    try:
        return api_service.get_cortical_area_list()
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
```

## External Client Usage

### REST API

The REST API is the primary interface for administrative operations and configuration.

#### Example: Get all cortical areas

```python
import requests

url = "http://localhost:8000/v1/cortical_area/cortical_area_id_list"
response = requests.get(url)
cortical_areas = response.json()
print(cortical_areas)
```

#### Example: Create a new cortical area

```python
import requests

url = "http://localhost:8000/v1/cortical_area/cortical_area"
data = {
    "name": "My Test Area",
    "cortical_id": "TestA1",
    "dimensions": {"x": 64, "y": 64, "z": 1},
    "coordinates": {"x": 100, "y": 100, "z": 0}
}
response = requests.post(url, json=data)
print(response.status_code)
```

### ZeroMQ Streams

ZeroMQ streams provide high-performance, real-time communication for sensorimotor data and visualization.

#### Example: Sending sensory data

```python
import zmq
import time
from feagi_bytes import ByteStructureTranslator

# Initialize ZMQ context and socket
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect("tcp://localhost:5557")

# Create a translator
translator = ByteStructureTranslator()

# Prepare neuron data
neuron_data = {
    (0, 0, 0): 0.8,  # Position (x,y,z): activation value
    (1, 0, 0): 0.6,
    (0, 1, 0): 0.7
}

# Encode data
cortical_id = "iv00_C"
encoded = translator.encode_neuron_data(cortical_id, neuron_data)

# Send data
socket.send(encoded)

# Clean up
socket.close()
context.term()
```

#### Example: Receiving motor data

```python
import zmq
import time
from feagi_bytes import ByteStructureTranslator

# Initialize ZMQ context and socket
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5558")
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

# Create a translator
translator = ByteStructureTranslator()

# Receive and decode data
while True:
    try:
        message = socket.recv(flags=zmq.NOBLOCK)
        cortical_id, data = translator.decode_neuron_data(message)
        print(f"Received data from {cortical_id}: {data}")
    except zmq.Again:
        time.sleep(0.01)
```

### Client Libraries

For more convenient access, use the FEAGI client libraries:

#### Python Client

```python
from feagi_connector import FeagiClient

async def main():
    # Connect to FEAGI
    client = FeagiClient(host="localhost")
    await client.connect()
    
    # Send sensory data
    neuron_data = {
        (0, 0, 0): 0.8,
        (1, 0, 0): 0.6,
        (0, 1, 0): 0.7
    }
    await client.send_sensory_data("iv00_C", neuron_data)
    
    # Register a callback for motor data
    @client.on_motor_data
    async def handle_motor(cortical_id, data):
        print(f"Motor data from {cortical_id}: {data}")
    
    # Wait for motor data
    await asyncio.sleep(10)
    
    # Disconnect
    await client.disconnect()
```

## API Versioning

The REST API follows semantic versioning with stable paths:

- `/v1/...` - Stable API, backwards compatible
- `/v2/...` - Updated API, may have breaking changes

## Best Practices

1. **Always check for errors** and handle them appropriately
2. **Use the newest API version** for new development
3. **Monitor connection state** for ZeroMQ streams
4. **Use exponential backoff** for reconnection attempts
5. **Implement proper error handling** for all API calls

## Related Documentation

- [API Module README](README.md)
- [API Architecture Decision Record](../../docs/adr-api-refactoring.md)
- [API Formats Specification](../../docs/spec-api-formats.md) 