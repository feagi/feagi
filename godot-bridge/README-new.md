# FEAGI Godot Bridge

This bridge enables real-time visualization and interaction between FEAGI and the Godot game engine. The bridge visualizes neural activity, cortical areas, and provides an interactive interface for monitoring and interacting with FEAGI.

## Features

- Real-time neural activity visualization
- Connectome structure visualization
- Interactive cortical area inspection
- Direct stimulation of neurons
- Support for standard REST API and ZMQ protocols
- Configurable bridge settings

## Communication Methods

The Godot Bridge can communicate with FEAGI using two different methods:

1. **Legacy HTTP REST API**: The original communication method using HTTP for control commands.
2. **ZMQ REST API**: A more efficient method using ZeroMQ that mirrors the REST API format.

## Requirements

- Python 3.8+
- FEAGI 2.1+
- WebSockets support
- ZeroMQ

## Installation

1. Clone the repository or navigate to the `godot-bridge` directory in your FEAGI installation.
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Configuration

Configuration is stored in `configuration.json`:

```json
{
  "feagi_settings": {
    "feagi_host": "127.0.0.1",
    "feagi_api_port": "8000",
    "feagi_zmq_port": "5559",
    "feagi_viz_port": "5560"
  },
  "agent_settings": {
    "godot_websocket_port": "9050"
  }
}
```

You can override these settings with environment variables:
- `FEAGI_HOST_INTERNAL` - FEAGI host address
- `FEAGI_API_PORT` - HTTP REST API port 
- `FEAGI_ZMQ_PORT` - ZMQ control port
- `FEAGI_VIZ_PORT` - ZMQ visualization port
- `WS_BRIDGE_PORT` - WebSocket port for Godot

## Usage

### Using the ZMQ REST API Bridge (Recommended)

The ZMQ REST API bridge provides better performance and a more efficient protocol:

```bash
python godot_zmq_rest_bridge.py
```

Options:
- `--config` - Path to configuration file (default: `configuration.json`)
- `--log-level` - Logging level: DEBUG, INFO, WARNING, ERROR, CRITICAL (default: INFO)

### Using the Legacy Bridge

The legacy bridge uses the HTTP REST API:

```bash
python bridge_godot_python_new.py
```

## Testing

Several test scripts are available to verify functionality:

- `test_zmq_rest_api.py`: Test the ZMQ REST API client
- `check_feagi_api.py`: Check which API endpoints are available
- `test_direct_fcl_injection.py`: Test direct FCL injection
- `test_sensory_input.py`: Test sending sensory input to FEAGI

## Godot Integration

The Godot game engine connects to this bridge via WebSocket. Your Godot project needs a WebSocket client implementation that can:

1. Connect to the bridge's WebSocket server (default port 9050)
2. Send and receive JSON messages
3. Process binary frame data (for neural activity visualization)

## Contributing

Contributions are welcome! Please follow the project's coding guidelines.

## License

Licensed under the Apache License, Version 2.0. 