# FEAGI Connector Usage Guide

FEAGI Connector is a client-side library that enables agents to connect to FEAGI and exchange data using its communication protocols. This guide explains how to use the connector in your agent implementations.

## Basic Concepts

FEAGI Connector provides:

1. **Agent Registration**: Register your agent with FEAGI
2. **Sensory Data Transmission**: Send sensory inputs to FEAGI
3. **Motor Data Reception**: Receive motor outputs from FEAGI
4. **Visualization**: Receive neural activity and brain structure data
5. **Connection Management**: Maintain connection with heartbeats

## Getting Started

### Installation

```bash
pip install feagi_connector
```

### Simple Agent Example

```python
import asyncio
from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel

async def main():
    # Create a client
    client = FeagiClient(
        host="localhost",
        agent_id="my-agent",
        agent_type="example"
    )
    
    # Connect to FEAGI
    connected = await client.connect()
    if not connected:
        print("Failed to connect to FEAGI")
        return
        
    # Register a callback for motor data
    await client.register_motor_callback(handle_motor_data)
    
    # Send some sensory data
    image_data = bytes([0x80] * (10 * 10))  # 10x10 grayscale image
    await client.send_sensory_data(FSMPChannel.VISION, image_data)
    
    # Keep the connection alive for a while
    await asyncio.sleep(10)
    
    # Disconnect
    await client.disconnect()
    
def handle_motor_data(channel_id, data):
    print(f"Received motor data on channel {channel_id}: {len(data)} bytes")

if __name__ == "__main__":
    asyncio.run(main())
```

## Core Features

### Agent Registration

When you connect to FEAGI, the connector automatically registers your agent:

```python
client = FeagiClient(
    host="localhost", 
    agent_id="my-agent",  # Optional, auto-generated if not provided
    agent_type="vision"   # Agent type for categorization
)

# Connect and register
connected = await client.connect()
```

### Sending Sensory Data

Send sensory inputs to FEAGI using the appropriate channel:

```python
# Send vision data
await client.send_sensory_data(FSMPChannel.VISION, image_bytes)

# Send proprioception data
await client.send_sensory_data(FSMPChannel.PROPRIOCEPTION, position_bytes)

# Send audio data
await client.send_sensory_data(FSMPChannel.AUDIO, audio_bytes)
```

### Receiving Motor Data

Register a callback to process motor outputs from FEAGI:

```python
async def start_agent():
    # Register callback
    await client.register_motor_callback(process_motor_data)
    
def process_motor_data(channel_id, data):
    if channel_id == FSMPChannel.MOTOR_ARM:
        # Process arm movement commands
        arm_position = decode_arm_position(data)
        move_robot_arm(arm_position)
    elif channel_id == FSMPChannel.MOTOR_SPEECH:
        # Process speech output
        speech_data = decode_speech(data)
        generate_audio(speech_data)
```

### Visualization Data

Receive neural activity and brain structure data for visualization:

```python
await client.register_visualization_callbacks(
    activity_callback=handle_activity_data,
    structure_callback=handle_structure_data
)

def handle_activity_data(data):
    # Process neural activity data
    # This is binary data in the FEAGI Bytes format
    # Use feagi_bytes to decode if needed
    activity_map = decode_activity_data(data)
    update_visualization(activity_map)
    
def handle_structure_data(data):
    # Process brain structure data
    brain_structure = decode_structure_data(data)
    update_3d_model(brain_structure)
```

### Connection Management

Maintain the connection with periodic heartbeats:

```python
async def heartbeat_loop():
    while running:
        await client.send_heartbeat()
        await asyncio.sleep(5)  # Send heartbeat every 5 seconds
```

## Advanced Usage

### Custom Transport

By default, FEAGI Connector uses ZeroMQ for communication. You can specify this explicitly:

```python
client = FeagiClient(
    host="localhost",
    transport="zmq",  # Default
    zmq_port=5570     # Optional, default is 5570
)
```

### Getting FEAGI Status

Retrieve the current status of FEAGI:

```python
status = await client.get_status()
print(f"FEAGI version: {status.get('version')}")
print(f"Running: {status.get('running')}")
print(f"Connected agents: {status.get('agents')}")
```

### Error Handling

Implement proper error handling for robustness:

```python
try:
    connected = await client.connect()
    if not connected:
        # Handle connection failure
        print("Failed to connect to FEAGI")
        return
        
    # Use the connection...
    
except Exception as e:
    print(f"Error: {e}")
    
finally:
    # Always disconnect properly
    await client.disconnect()
```

## Complete Agent Implementation

Here's a more complete agent implementation:

```python
import asyncio
import logging
from feagi_connector import FeagiClient
from feagi_connector.protocols import FSMPChannel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent")

class FeagiAgent:
    def __init__(self, host="localhost"):
        self.client = FeagiClient(
            host=host,
            agent_id="my-agent",
            agent_type="example"
        )
        self.running = False
        
    async def start(self):
        # Connect to FEAGI
        connected = await self.client.connect()
        if not connected:
            logger.error("Failed to connect to FEAGI")
            return False
            
        # Register callbacks
        await self.client.register_motor_callback(self._handle_motor)
        await self.client.register_visualization_callbacks(
            activity_callback=self._handle_activity,
            structure_callback=self._handle_structure
        )
        
        self.running = True
        logger.info("Agent started")
        
        # Start heartbeat loop
        asyncio.create_task(self._heartbeat_loop())
        return True
        
    async def stop(self):
        self.running = False
        await self.client.disconnect()
        logger.info("Agent stopped")
        
    async def _heartbeat_loop(self):
        while self.running:
            await self.client.send_heartbeat()
            await asyncio.sleep(5)
            
    def _handle_motor(self, channel_id, data):
        logger.info(f"Motor data on channel {channel_id}: {len(data)} bytes")
        # Process motor data...
        
    def _handle_activity(self, data):
        logger.info(f"Activity data: {len(data)} bytes")
        # Process activity data...
        
    def _handle_structure(self, data):
        logger.info(f"Structure data: {len(data)} bytes")
        # Process structure data...
        
    async def send_sensor_data(self, channel_id, data):
        if self.running:
            await self.client.send_sensory_data(channel_id, data)
            logger.info(f"Sent sensor data on channel {channel_id}")

# Usage
async def main():
    agent = FeagiAgent(host="localhost")
    try:
        if await agent.start():
            # Send some data
            for i in range(10):
                data = bytes([i % 256] * 100)
                await agent.send_sensor_data(FSMPChannel.VISION, data)
                await asyncio.sleep(1)
    finally:
        await agent.stop()

if __name__ == "__main__":
    asyncio.run(main())
```

## Channel Reference

FEAGI uses numeric channel IDs for different sensory and motor pathways:

| Channel | ID | Description |
|---------|-----|-------------|
| VISION | 1 | Visual input |
| AUDIO | 2 | Audio input |
| PROPRIOCEPTION | 3 | Body position sense |
| TOUCH | 4 | Touch sensors |
| OLFACTORY | 5 | Smell sensors |
| TASTE | 6 | Taste sensors |
| MOTOR_ARM | 101 | Arm movement |
| MOTOR_LEG | 102 | Leg movement |
| MOTOR_SPEECH | 103 | Speech output |
| MOTOR_EYES | 104 | Eye movement |

## Performance Tips

1. **Minimize Data Size**: Send only necessary data to reduce bandwidth
2. **Batch Updates**: Group related sensory data when possible
3. **Async Processing**: Process motor callbacks asynchronously to avoid blocking
4. **Error Recovery**: Implement reconnection logic for network issues 