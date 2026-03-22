# Servo Motor Example

Demonstrates ServoMotor usage with FEAGI for positional control:

- Registering multiple servos with different ranges (0-180, 0-270, -90 to +90 deg)
- Connecting to FEAGI via ZMQ
- Reading servo angles in the main loop and applying to hardware

## Requirements

- Python 3.10+
- See `requirements.txt` in this folder.

## Configuration

Connection uses environment variables. Required: `FEAGI_AGENT_DESCRIPTOR_B64`, `FEAGI_HOST`, `FEAGI_REGISTRATION_PORT`, `FEAGI_SENSORY_PORT`, `FEAGI_MOTOR_PORT`, `FEAGI_CONNECTION_TIMEOUT_MS`, `FEAGI_REGISTRATION_RETRIES`, `FEAGI_HEARTBEAT_INTERVAL_S`.

## Run

From this folder (venv activated, env set):

```bash
python example_servo_motor.py
```

FEAGI must be running; motor output port (e.g. 5564) must be accessible.
