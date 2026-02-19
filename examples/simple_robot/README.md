# Simple Robot Example

Demonstrates the FEAGI SDK 2.0 inputs/outputs API for a simple robot with:

- Camera (640x480 visual input)
- Infrared sensor (distance 0-400 cm)
- Servo motor (0-180 deg)
- Rotary motors (bidirectional wheels)

Shows how to register sensors and motors, configure and connect to FEAGI, send sensor data, and receive motor commands.

## Requirements

- Python 3.10+
- See `requirements.txt` in this folder.

## Configuration

Connection uses environment variables (no hardcoded hosts/ports). Optionally use `feagi_configuration.toml` from the parent `examples/` folder or set `FEAGI_CONFIG_PATH`.

Required env: `FEAGI_AGENT_DESCRIPTOR_B64`, `FEAGI_HOST`, `FEAGI_REGISTRATION_PORT`, `FEAGI_SENSORY_PORT`, `FEAGI_MOTOR_PORT`, `FEAGI_CONNECTION_TIMEOUT_MS`, `FEAGI_REGISTRATION_RETRIES`, `FEAGI_HEARTBEAT_INTERVAL_S`.

## Run

From this folder (with venv activated and env set):

```bash
python example_simple_robot.py
```

Or from the SDK root with PYTHONPATH:

```bash
cd feagi-python-sdk/examples/simple_robot
python example_simple_robot.py
```

FEAGI must be running; `feagi_rust_py_libs` is required for the PNS client.
