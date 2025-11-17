---
sidebar_position: 3
---

# Connecting Agents to FEAGI

FEAGI supports connections to external agents and systems through a variety of protocols. This guide explains how to connect different types of agents to your FEAGI brain.

## What are FEAGI Agents?

Agents are external components that interface with FEAGI, allowing the neural network to interact with the outside world. They can serve as:

- **Sensors**: Providing input to the brain (cameras, microphones, etc.)
- **Effectors**: Receiving output from the brain (motors, displays, etc.)
- **Processing Units**: Specialized systems that enhance brain capabilities

## Connection Methods

### ZeroMQ (ZMQ) Connection

ZeroMQ provides high-performance, asynchronous messaging for agent connections:

```python
import zmq
import json

# Connect to FEAGI
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5570")

# Send data to FEAGI
message = {
    "agent_name": "my_camera",
    "cortical_area": "visual_input",
    "data": [[0, 1, 0], [1, 0, 1], [0, 1, 0]]
}
socket.send_json(message)

# Receive response
response = socket.recv_json()
print(response)
```

### FEAGI Connector Library

The FEAGI Connector is a Python library that simplifies agent development:

1. **Install the library**:

```bash
pip install feagi-connector
```

2. **Create a simple agent**:

```python
from feagi_connector_old import FeagiConnector

# Create connector
connector = FeagiConnector(
    feagi_host="localhost",
    feagi_api_port=8000,
    agent_name="my_agent",
    sensory_mapping={"camera": "visual_input"},
    motor_mapping={"action": "motor_output"}
)

# Define callback for brain output
def process_brain_output(data):
    print(f"Received from brain: {data}")
    # Process motor commands here

# Register callback
connector.register_motor_callback(process_brain_output)

# Start the connection
connector.connect()

# Send sensory data
visual_data = create_image_data()  # Your function to create input data
connector.send_sensory_data("camera", visual_data)
```

### WebSocket Connection

For web applications, you can use WebSocket connections:

```javascript
// Connect to FEAGI
const socket = new WebSocket('ws://localhost:9000/ws/agents');

// Handle connection opening
socket.onopen = function(event) {
  console.log('Connected to FEAGI');

  // Register agent
  socket.send(JSON.stringify({
    type: 'register',
    name: 'web_interface',
    input_areas: ['visual_input'],
    output_areas: ['text_output']
  }));
};

// Handle incoming messages
socket.onmessage = function(event) {
  const data = JSON.parse(event.data);
  console.log('Received data:', data);
  // Process brain output
};

// Send data to brain
function sendToBrain(data) {
  socket.send(JSON.stringify({
    type: 'sensory_data',
    area: 'visual_input',
    data: data
  }));
}
```

## FEAGI Bytes Protocol

FEAGI communicates with agents using the FEAGI Bytes protocol:

```python
from feagi_bytes import FeagiBytes

# Create encoder/decoder
fb = FeagiBytes()

# Encode data for transmission
encoded_data = fb.encode_array(my_numpy_array)

# Decode received data
decoded_data = fb.decode_array(received_bytes)
```

## Example Agent Applications

### Camera Input Agent

```python
import cv2
from feagi_connector_old import FeagiConnector

# Initialize camera
cap = cv2.VideoCapture(0)

# Create FEAGI connector
feagi = FeagiConnector(
    feagi_host="localhost",
    feagi_api_port=8000,
    agent_name="camera_agent",
    sensory_mapping={"camera": "visual_input"}
)

# Connect to FEAGI
feagi.connect()

while True:
    # Capture frame
    ret, frame = cap.read()
    if not ret:
        break

    # Process frame (resize, convert to grayscale, normalize)
    processed_frame = process_frame(frame)

    # Send to FEAGI
    feagi.send_sensory_data("camera", processed_frame)

    # Break loop on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
feagi.disconnect()
```

## Next Steps

- Check out the [Tutorials](/user-guide/tutorials) for more examples
- Learn about [REST API](/modules/api/guide-api-usage) for programmatic control
- Explore our [pre-built agents](https://github.com/feagi/feagi-agents) on GitHub
