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
        # agent_id must be a base64 AgentDescriptor (48-byte payload)
        agent_id="<agent_descriptor_b64>",
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
    # agent_id must be a base64 AgentDescriptor (48-byte payload)
    agent_id="<agent_descriptor_b64>",
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
    if channel_id == FSMPChannel.MOTOR_ARM.value:
        # Process arm movement commands
        arm_position = decode_arm_position(data)
        move_robot_arm(arm_position)
    elif channel_id == FSMPChannel.MOTOR_SPEECH.value:
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
    # This is a dictionary of neuron coordinates to activation values
    activity_map = data.get("data", {})
    update_visualization(activity_map)
    
def handle_structure_data(data):
    # Process brain structure data
    brain_structure = data.get("data", {})
    update_3d_model(brain_structure)
```

### Connection Management

The client automatically maintains the connection with periodic heartbeats when you call `connect()`. You can disconnect cleanly with:

```python
await client.disconnect()
```

## Advanced Usage

### Custom Port Configuration

By default, FEAGI Connector uses standard ports for different protocols. You can specify these explicitly:

```python
client = FeagiClient(
    host="localhost",
    command_port=5555,  # REQ/REP for commands
    sensory_port=5558,  # DEALER/ROUTER for sensory/motor data
    viz_port=5562       # DEALER/ROUTER for visualization data
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
from feagi_connector.utils import setup_logging

# Configure logging
setup_logging(level=logging.INFO, log_file="agent.log")
logger = logging.getLogger("agent")

class FeagiAgent:
    def __init__(self, host="localhost"):
        self.client = FeagiClient(
            host=host,
            # agent_id must be a base64 AgentDescriptor (48-byte payload)
            agent_id="<agent_descriptor_b64>",
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
        return True
        
    async def stop(self):
        self.running = False
        await self.client.disconnect()
        logger.info("Agent stopped")
        
    async def send_image(self, image_data):
        """Send an image to FEAGI."""
        return await self.client.send_sensory_data(
            FSMPChannel.VISION, 
            image_data
        )
        
    async def _handle_motor(self, channel_id, data):
        """Handle motor data from FEAGI."""
        logger.info(f"Received motor data on channel {channel_id}: {len(data)} bytes")
        # Process motor data...
        
    async def _handle_activity(self, data):
        """Handle neural activity data from FEAGI."""
        logger.info(f"Received activity data: {len(data.get('data', {}))} neurons")
        # Process activity data...
        
    async def _handle_structure(self, data):
        """Handle brain structure data from FEAGI."""
        logger.info(f"Received brain structure data")
        # Process structure data...

async def main():
    agent = FeagiAgent(host="localhost")
    if await agent.start():
        try:
            # Generate test image
            image_data = bytes([0x80] * (10 * 10))  # 10x10 grayscale image
            
            # Send image every second for 30 seconds
            for _ in range(30):
                await agent.send_image(image_data)
                await asyncio.sleep(1)
                
        finally:
            await agent.stop()

if __name__ == "__main__":
    asyncio.run(main())
```

## Troubleshooting

### Connection Issues

If you're having trouble connecting to FEAGI:

1. **Check that FEAGI is running**: Use the command client to ping FEAGI
   ```python
   status = await client.command_client.ping()
   print(f"FEAGI status: {status}")
   ```

2. **Verify port configuration**: Make sure you're using the correct ports for each protocol

3. **Check network connectivity**: Ensure there are no firewalls blocking the connection

### Data Transmission Issues

If sensory data isn't being processed by FEAGI:

1. **Check registration**: Make sure your agent is properly registered
   ```python
   if not client.sensory_client.registered:
       await client.sensory_client.register_agent()
   ```

2. **Verify cortical area**: Ensure you're sending to a valid cortical area

3. **Check data format**: Make sure your data is correctly formatted (bytes or neuron dictionary) 