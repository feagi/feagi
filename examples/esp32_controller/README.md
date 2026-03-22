# ESP32 Serial Controller Example

Bridges ESP32 firmware communication with FEAGI using `Esp32SerialController`:

- Auto-detects ESP32 serial port (or use `--serial-port`)
- Connects to FEAGI via ZMQ
- Bridges sensory data ESP32 to FEAGI and motor commands FEAGI to ESP32

## Requirements

- Python 3.10+
- See `requirements.txt`. Uses pyserial (included with feagi-core).

## Prerequisites

- ESP32 connected via USB with FEAGI controller firmware
- FEAGI running (localhost or set `--feagi-host`)

## Run

From this folder:

```bash
# Auto-detect ESP32 port
python esp32_controller_example.py --agent-id esp32-01

# Specify port
python esp32_controller_example.py --agent-id esp32-01 --serial-port /dev/ttyUSB0

# Custom FEAGI host (set via env or config in production; example uses CLI for clarity)
python esp32_controller_example.py --agent-id esp32-01 --feagi-host 192.168.1.100
```
