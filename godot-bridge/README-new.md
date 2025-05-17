# FEAGI 2.0 Godot Bridge

This bridge connects FEAGI 2.0 to the Godot game engine for 3D visualization of neural activity.

## Overview

The FEAGI 2.0 Godot Bridge subscribes to FEAGI's visualization data stream, processes neural activity data, and forwards it to Godot for real-time visualization. It serves as a critical component for monitoring and debugging neural networks running in FEAGI.

## Features

- Real-time visualization of neuron firing patterns
- Performance optimizations for handling large neural networks
- Caching of coordinate data to minimize CPU usage
- WebSocket interface for Godot communication
- Support for direct neuron stimulation from Godot

## Requirements

- Python 3.9+
- FEAGI 2.0+
- Godot 4.0+ with the FEAGI Brain Visualizer project

## Installation

1. Clone the FEAGI repository
2. Navigate to the godot-bridge directory
3. Install dependencies: `pip install -r requirements.txt`

## Usage

### Running the Bridge

```bash
python bridge_godot_python_new.py
```

### Configuration

The bridge can be configured using the `configuration.json` file or environment variables:

| Setting | Environment Variable | Default | Description |
|---------|---------------------|---------|-------------|
| FEAGI Host | FEAGI_HOST_INTERNAL | 127.0.0.1 | IP address or hostname of FEAGI |
| FEAGI API Port | FEAGI_API_PORT | 8000 | FEAGI API port |
| WebSocket Port | WS_BRIDGE_PORT | 9050 | Port for Godot WebSocket connection |

### Godot Integration

In your Godot project, connect to the bridge using the WebSocket protocol:

```gdscript
func _ready():
    var client = WebSocketClient.new()
    client.connect_to_url("ws://localhost:9050")
    # Implement handlers for receiving visualization data
```

## Architecture

The bridge consists of several components:

1. **FEAGI Client**: Connects to FEAGI using the ZMQ protocol
2. **Data Processor**: Transforms neuron firing data for Godot visualization
3. **WebSocket Server**: Provides real-time data to Godot
4. **Command Handler**: Processes stimulation commands from Godot

## Performance Considerations

The bridge is optimized for performance with large neural networks:

- Coordinate caching reduces CPU usage for repeated patterns
- Pre-allocated buffers minimize memory allocations
- Rate limiting prevents overwhelming the WebSocket connection

## Troubleshooting

- **Connection Issues**: Verify FEAGI is running and accessible
- **No Visualization**: Check if the visualization stream is enabled in FEAGI
- **High Latency**: Reduce the number of neurons or increase stimulation period

## Contributing

Contributions are welcome! Please follow the FEAGI contribution guidelines.

## License

Licensed under the Apache License, Version 2.0. 