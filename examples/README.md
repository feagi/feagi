# FEAGI Python SDK - Examples

Each example lives in its own folder with a README and optional `requirements.txt`. Use the SDK virtual environment or install dependencies per example.

## Requirements

- Python 3.10+
- FEAGI Python SDK. From repo root: `pip install -e ".[video]"` (or use project venv). Optional: `feagi_rust_py_libs` for PNS client performance.

Configuration is read from `feagi_configuration.toml` in this directory (or from paths set by your environment). Do not hardcode hosts or ports; use environment variables or the config file.

## Overview

| Example Folder | Description |
|----------------|-------------|
| `simple_robot/` | Robot with camera, infrared, servo, rotary motors; `feagi.pns` inputs/outputs |
| `servo_motor/` | Servo motor control only; register servos, connect, read angles in a loop |
| `video_streamer/` | Video streaming with `FeagiEngine` and `VideoStreamAgent` |
| `genome_vs_connectome/` | Difference between loading a genome vs a connectome with `FeagiEngine` |
| `esp32_controller/` | ESP32 serial bridge using `feagi.agent.esp32.Esp32SerialController` |
| `mixed_transport_agent/` | Agent that chooses ZMQ or WebSocket based on FEAGI registration |
| `observability/` | PNS observability: metrics, logging, profiling, validation |

## Running Examples

Each folder has its own README and (where applicable) `requirements.txt`. Run from the example folder or set `PYTHONPATH` to include the SDK.

```bash
cd simple_robot
python example_simple_robot.py
```

Or from this directory:

```bash
python simple_robot/example_simple_robot.py
```

Common environment variables (no hardcoded defaults): `FEAGI_HOST`, `FEAGI_AGENT_DESCRIPTOR_B64`, `FEAGI_REGISTRATION_PORT`, `FEAGI_SENSORY_PORT`, `FEAGI_MOTOR_PORT`. See each example README for full list.

## Shared Configuration

- **feagi_configuration.toml** – Used by engine and video/genome examples. Stored here; example folders resolve it (this folder or parent). Override with `FEAGI_CONFIG_PATH` where supported.
- **agent_config.toml.template** – Sample agent config; copy to your agent directory and customize.

## Troubleshooting

- **FEAGI logs "Unknown cortical area" (base64 IDs)** – The genome's cortical area IDs do not match the IDs the agent sends. Use a genome that defines vision (IPU) areas compatible with the agent's pipeline, or align registration and genome (see FEAGI/BDU docs).

## SDK Modules Used

- **feagi.engine** – `FeagiEngine` for starting/stopping the neural engine and loading config/genome/connectome
- **feagi.agent** – `VideoStreamAgent`, `Esp32SerialController`
- **feagi.pns** – `brain_input`, `brain_output`, `FeagiAgentClient`, `AgentType`; inputs (Camera, Infrared), outputs (ServoMotor, RotaryMotor)
- **feagi.pns.observability** – `enable_monitoring`, MetricsCollector, DataLogger, DataInspector, Profiler

For full API details, see the SDK package documentation and `feagi/` source.
