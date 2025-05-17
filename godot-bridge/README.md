# FEAGI 2.0 Godot Bridge

This bridge enables visualization of FEAGI 2.0 neural activity in Godot game engine. It connects to FEAGI using the ZMQ visualization stream API, processes neuron activity data, and forwards it to Godot for 3D visualization.

## Features

- Real-time 3D visualization of neuron firing activity
- Visualization of cortical areas and their dimensions
- Support for updating the visualization when the genome changes
- High-performance data processing with optimizations for large neural networks
- WebSocket-based communication with Godot

## Requirements

- Python 3.9 or higher
- FEAGI 2.0
- Godot 3.5 or higher
- Python dependencies (listed in requirements.txt)

## Installation

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Configure the bridge (see Configuration section below)

3. Start the bridge:
   ```bash
   python bridge_godot_python_new.py
   ```

## Configuration

The bridge is configured using a `configuration.json` file with the following structure:

```json
{
  "feagi_settings": {
    "feagi_host": "127.0.0.1",
    "feagi_api_port": "8000",
    "feagi_auth_token": ""
  },
  "agent_settings": {
    "godot_websocket_ip": "0.0.0.0",
    "godot_websocket_port": "9050"
  }
}
```

### Configuration Parameters

#### FEAGI Settings
- `feagi_host`: FEAGI host address
- `feagi_api_port`: FEAGI REST API port
- `feagi_auth_token`: Authentication token (if required)

#### Agent Settings
- `godot_websocket_ip`: IP address for the WebSocket server
- `godot_websocket_port`: Port for the WebSocket server

### Environment Variables

The following environment variables can override the configuration:

- `FEAGI_HOST_INTERNAL`: Override FEAGI host
- `FEAGI_API_PORT`: Override FEAGI API port
- `WS_BRIDGE_PORT`: Override WebSocket port

## Usage with Godot

In Godot, connect to the bridge using the WebSocket client with the configured IP and port. The bridge sends neuron activity data in binary format, which can be decoded to visualize neurons as 3D objects in Godot.

Example code for connecting to the bridge from Godot:

```gdscript
# In Godot script
var websocket_client = WebSocketClient.new()

func _ready():
    # Connect to WebSocket
    var url = "ws://127.0.0.1:9050"
    websocket_client.connect("connection_established", self, "_on_connection_established")
    websocket_client.connect("connection_error", self, "_on_connection_error")
    websocket_client.connect("connection_closed", self, "_on_connection_closed")
    websocket_client.connect("data_received", self, "_on_data_received")
    
    var error = websocket_client.connect_to_url(url)
    if error != OK:
        print("Error connecting to websocket server: ", error)

func _process(delta):
    # Poll for WebSocket events
    websocket_client.poll()

func _on_connection_established(protocol):
    print("Connected to bridge!")

func _on_data_received():
    # Process received data
    var data = websocket_client.get_peer(1).get_packet()
    var decompressed_data = decompress_data(data)
    process_visualization_data(decompressed_data)

func decompress_data(data):
    return data.decompress(StreamPeerBuffer.COMPRESSION_DEFLATE)

func process_visualization_data(data):
    # Implement visualization logic here
    # See the documentation for data format details
```

## Data Format

The bridge sends two main types of data to Godot:

1. **Status Data**: JSON data containing FEAGI status (genome loaded, brain running)
2. **Neuron Activity Data**: Binary data for each cortical area with neuron coordinates

## Development

To extend the bridge functionality:

1. Make changes to `bridge_godot_python_new.py`
2. Test your changes with both FEAGI and Godot
3. Check log output for errors and performance information

## Troubleshooting

- **Connection issues**: Verify FEAGI is running and accessible. Check firewall settings.
- **No data received**: Ensure FEAGI has a genome loaded and the brain is running.
- **Performance problems**: Check the console for performance metrics. Adjust sampling rates if needed.

## License

Copyright 2023-Present The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0