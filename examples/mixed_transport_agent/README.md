# Mixed Transport Agent Example

Shows how to build an agent that can use either ZMQ or WebSocket based on FEAGI's available transports:

1. Register with FEAGI
2. Parse available transports from the registration response
3. Select transport (auto, prefer WebSocket, force ZMQ, or browser scenario)
4. Connect with the chosen transport

## Requirements

- Python 3.10+
- See `requirements.txt`. For WebSocket scenario: `pip install websockets` (optional).

## Configuration

All connection parameters come from environment variables (no hardcoded defaults):

- `FEAGI_API_HOST`, `FEAGI_API_PORT`
- `AGENT_DESCRIPTOR_B64`, `AGENT_TYPE`, `AGENT_CAPABILITIES_JSON`
- `AGENT_DATA_PORT`, `AGENT_VERSION`, `CONTROLLER_VERSION`
- `FEAGI_HEARTBEAT_INTERVAL_S`, `FEAGI_CONNECTION_TIMEOUT_MS`, `FEAGI_REGISTRATION_RETRIES`
- `FEAGI_AUTH_TOKEN_B64`

## Run

From this folder with env set:

```bash
python mixed_transport_agent.py
```

FEAGI must be running. The example demonstrates transport selection; Scenario C performs a real ZMQ connect if FEAGI is reachable.
