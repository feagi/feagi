# FEAGI Python SDK - Examples

This directory contains runnable examples that use the current FEAGI Python SDK (`feagi`). All examples are kept in sync with the latest SDK APIs.

## Requirements

- Python 3.10+
- FEAGI Python SDK and optional Rust backend:

```bash
# From project root, use the project virtual environment or:
pip install -e ".[video]"   # for video examples; feagi-core is the package name
# Optional: feagi_rust_py_libs is a dependency of feagi-core for PNS client performance
```

Configuration is read from `feagi_configuration.toml` in this folder (or from paths set by your environment). Do not hardcode hosts or ports; use environment variables or the config file.

## Overview

| Example | Description |
|---------|-------------|
| `example_simple_robot.py` | Robot with camera, infrared, servo, and rotary motors using `feagi.pns` inputs/outputs |
| `example_servo_motor.py` | Servo motor control only; register servos, connect, read angles in a loop |
| `example_video_simple.py` | Video streaming with `FeagiEngine` and `VideoStreamAgent` |
| `example_genome_vs_connectome.py` | Difference between loading a genome vs a connectome with `FeagiEngine` |
| `esp32_controller_example.py` | ESP32 serial bridge to FEAGI using `feagi.agent.esp32.Esp32SerialController` |
| `mixed_transport_agent.py` | Agent that chooses ZMQ or WebSocket based on FEAGI registration |
| `observability/` | PNS observability examples (metrics, logging, profiling, validation) |

## Running Examples

Run from the `examples` directory so that `feagi_configuration.toml` is found:

```bash
cd examples
python example_servo_motor.py
```

Several examples require environment variables for FEAGI connection (no hardcoded defaults per project rules). Typical variables:

- `FEAGI_HOST` – FEAGI server host
- `FEAGI_AGENT_DESCRIPTOR_B64` – Base64-encoded agent descriptor
- `FEAGI_REGISTRATION_PORT`, `FEAGI_SENSORY_PORT`, `FEAGI_MOTOR_PORT` – Ports (or use values from your `feagi_configuration.toml`)

Example (values must match your FEAGI config):

```bash
export FEAGI_HOST=localhost
export FEAGI_AGENT_DESCRIPTOR_B64=<your-agent-descriptor-base64>
export FEAGI_REGISTRATION_PORT=30001
export FEAGI_SENSORY_PORT=5555
export FEAGI_MOTOR_PORT=5564
python example_simple_robot.py
```

Engine and video examples that start FEAGI locally use the local `feagi_configuration.toml`:

```bash
python example_video_simple.py
python example_genome_vs_connectome.py
```

## Configuration

- **feagi_configuration.toml** – Used by engine and video examples in this folder. Copy or adapt for your deployment; override with environment variables where supported.
- **agent_config.toml.template** – Sample agent-side configuration template; copy to your agent directory and customize. Replace placeholder host/port values with your environment or config.

## Troubleshooting

- **FEAGI logs "Unknown cortical area" (base64 IDs)** – The genome's cortical area IDs do not match the IDs the agent sends (e.g. vision segments from the SDK). Use a genome that defines vision (IPU) areas compatible with the agent's pipeline, or align registration and genome (see FEAGI/BDU docs).

## Observability

The `observability/` subfolder contains examples for the PNS observability module:

| Script | Purpose |
|--------|---------|
| `01_basic_metrics.py` | MetricsCollector, export to JSON/CSV |
| `02_data_logging.py` | DataLogger, JSONL and CSV logging |
| `03_data_validation.py` | DataInspector, validation and anomaly detection |
| `04_performance_profiling.py` | Profiler, bottlenecks and optimization |
| `05_comprehensive_monitoring.py` | Combined use of observability components |
| `06_quick_start.py` | Minimal setup with `enable_monitoring()` |

Run from the examples directory:

```bash
python observability/01_basic_metrics.py
```

## SDK Modules Used

- **feagi.engine** – `FeagiEngine` for starting/stopping the neural engine and loading config/genome/connectome
- **feagi.agent** – `VideoStreamAgent`, `Esp32SerialController`
- **feagi.pns** – `brain_input`, `brain_output`, `FeagiAgentClient`, `AgentType`; `feagi.pns.inputs` (e.g. Camera, Infrared), `feagi.pns.outputs` (e.g. ServoMotor, RotaryMotor)
- **feagi.pns.observability** – `enable_monitoring`, MetricsCollector, DataLogger, DataInspector, Profiler

For full API details, see the SDK package documentation and `feagi/` source.
