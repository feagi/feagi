"""
Servo Motor Example using FEAGI SDK 2.0

Demonstrates how to use ServoMotor with FEAGI for positional control.
This example shows:
- Registering multiple servos with different ranges
- Connecting to FEAGI
- Reading servo angles in the main loop
- Applying angles to hardware
"""

import os
from feagi.pns.outputs import ServoMotor
from feagi.pns import brain_output
import time


def require_env(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"Missing required environment variable: {name}")
    return value


def parse_env_int(name: str) -> int:
    return int(require_env(name))


def parse_env_float(name: str) -> float:
    return float(require_env(name))


def main():
    print("=" * 60)
    print("FEAGI 2.0 - Servo Motor Example")
    print("=" * 60)

    # === Register Servos ===
    print("\nRegistering servo motors...")

    # Standard servo (0-180 deg)
    servo_head = ServoMotor.register(range=(0, 180), encoding="absolute")
    print("  Head servo registered (0-180 deg)")

    # Wide-range servo (0-270 deg)
    servo_arm = ServoMotor.register(range=(0, 270), encoding="absolute")
    print("  Arm servo registered (0-270 deg)")

    # Bidirectional servo (-90 to +90 deg)
    servo_tilt = ServoMotor.register(range=(-90, 90), encoding="absolute")
    print("  Tilt servo registered (-90 to +90 deg)")

    # === Configure Connection ===
    print("\nConfiguring connection...")

    agent_id = require_env("FEAGI_AGENT_DESCRIPTOR_B64")
    feagi_host = require_env("FEAGI_HOST")
    feagi_registration_port = parse_env_int("FEAGI_REGISTRATION_PORT")
    feagi_sensory_port = parse_env_int("FEAGI_SENSORY_PORT")
    feagi_motor_port = parse_env_int("FEAGI_MOTOR_PORT")
    feagi_connection_timeout_ms = parse_env_int("FEAGI_CONNECTION_TIMEOUT_MS")
    feagi_registration_retries = parse_env_int("FEAGI_REGISTRATION_RETRIES")
    feagi_heartbeat_interval_s = parse_env_float("FEAGI_HEARTBEAT_INTERVAL_S")

    brain_output.configure(
        agent_id=agent_id,
        feagi_host=feagi_host,
        feagi_registration_port=feagi_registration_port,
        feagi_sensory_port=feagi_sensory_port,
        feagi_motor_port=feagi_motor_port,
        transport="zmq",
        feagi_connection_timeout_ms=feagi_connection_timeout_ms,
        feagi_registration_retries=feagi_registration_retries,
        feagi_heartbeat_interval_s=feagi_heartbeat_interval_s,
    )

    print(
        f"  Configured: {feagi_host}:{feagi_motor_port} (ZMQ)"
    )

    # === Connect ===
    print("\nConnecting to FEAGI...")

    try:
        brain_output.connect()
        print("  Connected successfully!")
    except Exception as e:
        print(f"  Connection failed: {e}")
        print("\n  Make sure:")
        print("    1. FEAGI is running on localhost")
        print("    2. feagi_rust_py_libs is installed")
        print("    3. Motor output port 5564 is accessible")
        return

    # === Main Loop ===
    print("\nStarting motor control loop...")
    print("  (Press Ctrl+C to stop)\n")

    try:
        loop_count = 0
        while True:
            loop_count += 1

            # === Receive motor commands from FEAGI ===
            brain_output.receive()

            # === Read servo angles ===
            head_angle = servo_head.get_angle()
            arm_angle = servo_arm.get_angle()
            tilt_angle = servo_tilt.get_angle()

            # === Apply to hardware ===
            # In real code, replace with actual hardware API calls - see apply_servo_angle()
            apply_servo_angle("head", head_angle)
            apply_servo_angle("arm", arm_angle)
            apply_servo_angle("tilt", tilt_angle)

            # Print status every 10 loops
            if loop_count % 10 == 0:
                print(f"[Loop {loop_count:04d}] "
                      f"Head={head_angle:6.1f} deg | "
                      f"Arm={arm_angle:6.1f} deg | "
                      f"Tilt={tilt_angle:+6.1f} deg")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopping motor control...")

    # === Cleanup ===
    brain_output.disconnect()
    print("Disconnected")

    print("\n" + "=" * 60)
    print("Motor control stopped successfully!")
    print("=" * 60)


def apply_servo_angle(name: str, angle: float):
    """
    Simulated hardware function.

    In a real embodiment controller, this would call your
    actual servo control library (e.g., pigpio, Adafruit, etc.)
    """
    pass  # Placeholder for simulation


if __name__ == "__main__":
    main()
