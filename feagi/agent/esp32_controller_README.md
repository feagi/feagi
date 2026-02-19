# ESP32 Serial Controller

Python bridge that connects ESP32 firmware to FEAGI via serial communication and ZMQ.

## Overview

The `Esp32SerialController` acts as a **serial-to-ZMQ bridge** that:

1. **Reads** JSON sensory data from ESP32 via serial port
2. **Converts** to FEAGI neuron_id_potential_pairs format  
3. **Sends** to FEAGI via ZMQ PUSH (sensory input)
4. **Receives** motor commands from FEAGI via ZMQ SUB (motor output)
5. **Converts** to JSON format
6. **Writes** to ESP32 via serial port

## Architecture

```
┌──────────────┐      Serial/UART      ┌──────────────────────┐      ZMQ      ┌──────────────┐
│   ESP32      │ <────────────────────> │ Esp32SerialController │ <──────────> │    FEAGI     │
│  Firmware    │   JSON messages        │  (Python Bridge)      │              │              │
└──────────────┘                        └──────────────────────┘              └──────────────┘
```

## Message Formats

### ESP32 → FEAGI (Sensory Data)

**ESP32 Format:**
```json
{"np":[[123,1.0],[456,0.0]],"id":"esp32","f":42}
```

**Converted to FEAGI Format:**
```python
[(123, 1.0), (456, 0.0)]  # List of (neuron_id, potential) tuples
```

### FEAGI → ESP32 (Motor Commands)

**FEAGI Format:**
```python
{"motor": {"0": 50.0, "1": -30.0}}  # Motor index → power
```

**Converted to ESP32 Format:**
```json
{"motor_commands":[{"neuron_id":0,"value":50.0},{"neuron_id":1,"value":-30.0}]}
```

**Note on Motor Mapping:** Currently uses simple sequential mapping:
- Motor index 0 → neuron_id 0
- Motor index 1 → neuron_id 1
- etc.

This assumes GPIO outputs are configured with sequential neuron IDs. For advanced mapping, configure GPIO pins to use specific neuron IDs that match FEAGI's motor cortical areas.

## Usage

### Basic Usage

```python
from feagi.agent.esp32 import Esp32SerialController

controller = Esp32SerialController(
    # agent_id must be a base64 AgentDescriptor (48-byte payload)
    agent_id="<agent_descriptor_b64>",
    serial_port="/dev/ttyUSB0",  # or None for auto-detect
    baud_rate=115200,
    feagi_host="localhost"
)

controller.run()  # Blocks until Ctrl+C
```

### Command-Line Usage

```bash
# Auto-detect ESP32 port
python -m feagi.agent.esp32 --agent-id <agent_descriptor_b64>

# Specify serial port
python -m feagi.agent.esp32 --agent-id <agent_descriptor_b64> --serial-port /dev/ttyUSB0

# Custom FEAGI host
python -m feagi.agent.esp32 --agent-id <agent_descriptor_b64> --feagi-host 192.168.1.100
```

### Using the Example Script

```bash
python examples/esp32_controller/esp32_controller_example.py --agent-id <agent_descriptor_b64>
```

## Configuration

### Auto-Detection

The controller can auto-detect ESP32 serial ports by looking for:
- USB device descriptions containing "ESP32", "CH340", "CP210", "FTDI"
- Common Linux serial ports (`/dev/ttyUSB*`, `/dev/ttyACM*`)

### Port Detection

If multiple ESP32 devices are connected, specify the port manually:

```python
controller = Esp32SerialController(
    # agent_id must be a base64 AgentDescriptor (48-byte payload)
    agent_id="<agent_descriptor_b64>",
    serial_port="/dev/ttyUSB0",  # Explicit port
    auto_detect_port=False
)
```

### FEAGI Ports

Default ports (matches FEAGI 2.0 defaults):
- Registration: `30001`
- Sensory Input: `5555` (ZMQ PUSH)
- Motor Output: `5564` (ZMQ SUB)

## Error Handling

The controller includes:
- **Automatic retry** for serial read/write errors
- **Statistics tracking** (sensory sent, motor received, errors)
- **Graceful shutdown** on Ctrl+C
- **Thread-safe** operation (separate threads for sensory and motor)

## Statistics

The controller tracks:
- `sensory_messages_sent` - Number of sensory messages sent to FEAGI
- `motor_messages_received` - Number of motor commands received from FEAGI
- `serial_read_errors` - Serial port read errors
- `serial_write_errors` - Serial port write errors
- `feagi_errors` - FEAGI communication errors

Statistics are printed on shutdown.

## Troubleshooting

### No ESP32 Port Detected

```bash
# List available serial ports
python -c "import serial.tools.list_ports; [print(p.device) for p in serial.tools.list_ports.comports()]"

# Specify port manually
python -m feagi.agent.esp32 --agent-id esp32-01 --serial-port /dev/ttyUSB0
```

### FEAGI Connection Failed

- Ensure FEAGI is running: `feagi --check`
- Check FEAGI ports match controller configuration
- Verify firewall isn't blocking ZMQ ports

### Serial Communication Errors

- Check baud rate matches ESP32 firmware (default: 115200)
- Verify USB cable connection
- Check ESP32 firmware is running (look for serial output)

## Integration with ESP32 Flasher

This controller is designed to work with the ESP32 firmware built by the FEAGI Desktop ESP32 Flasher:

1. Use FEAGI Desktop to configure and flash ESP32 firmware
2. Run this controller to bridge serial communication to FEAGI
3. Controller automatically handles message format conversion

## Future Enhancements

- [ ] Configuration file support (TOML/YAML)
- [ ] Advanced motor index → neuron ID mapping from GPIO config
- [ ] Support for multiple ESP32 devices
- [ ] WebSocket transport support (alternative to serial)
- [ ] Binary protocol option (faster than JSON)

## Dependencies

- `pyserial>=3.5` - Serial port communication
- `feagi-rust-py-libs` - FEAGI ZMQ client (Rust-backed)
- ZMQ transport is handled by the Rust SDK (no Python ZMQ bindings required)

