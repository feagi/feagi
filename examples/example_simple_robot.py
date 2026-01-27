"""
Simple Robot Example using FEAGI SDK 3.0

Demonstrates the new inputs/outputs API for a simple robot with:
- Camera (640x480 visual input)
- Infrared sensor (distance sensing 0-400cm)
- Servo motor (positional control 0-180°)
- Rotary motors (bidirectional wheel control)

This example shows how to:
1. Register sensors and motors
2. Configure and connect to FEAGI
3. Send sensor data in a loop
4. Receive motor commands from FEAGI
"""

import os
from feagi.pns.inputs import Camera, Infrared
from feagi.pns.outputs import ServoMotor, RotaryMotor
from feagi.pns import brain_input, brain_output
import numpy as np
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
    print("FEAGI 3.0 - Simple Robot Example")
    print("=" * 60)
    
    # === Register Inputs ===
    print("\n📥 Registering inputs...")
    
    camera = Camera.register(
        resolution=(640, 480),
        encoding="absolute",
        position="center"
    )
    print(f"  ✓ Camera registered (640x480)")
    
    infrared_front = Infrared.register(
        encoding="absolute",
        inverted=False,
        min_distance=0.0,
        max_distance=400.0
    )
    print(f"  ✓ Infrared sensor registered (0-400cm)")
    
    # === Register Outputs ===
    print("\n📤 Registering outputs...")
    
    servo_head = ServoMotor.register(range=(0, 180))
    print(f"  ✓ Servo motor registered (0-180°)")
    
    motor_left = RotaryMotor.register(bidirectional=True)
    motor_right = RotaryMotor.register(bidirectional=True)
    print(f"  ✓ Rotary motors registered (left, right)")
    
    # === Configure Connection ===
    print("\n🔧 Configuring connection...")
    
    agent_id = require_env("FEAGI_AGENT_DESCRIPTOR_B64")
    feagi_host = require_env("FEAGI_HOST")
    feagi_registration_port = parse_env_int("FEAGI_REGISTRATION_PORT")
    feagi_sensory_port = parse_env_int("FEAGI_SENSORY_PORT")
    feagi_motor_port = parse_env_int("FEAGI_MOTOR_PORT")
    feagi_connection_timeout_ms = parse_env_int("FEAGI_CONNECTION_TIMEOUT_MS")
    feagi_registration_retries = parse_env_int("FEAGI_REGISTRATION_RETRIES")
    feagi_heartbeat_interval_s = parse_env_float("FEAGI_HEARTBEAT_INTERVAL_S")

    brain_input.configure(
        feagi_host=feagi_host,
        feagi_port=feagi_sensory_port,
        transport="zmq",
    )
    
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
        f"  ✓ Configured: {feagi_host}:{feagi_sensory_port} (input), "
        f"{feagi_host}:{feagi_motor_port} (output)"
    )
    
    # === Connect ===
    print("\n🔗 Connecting to FEAGI...")
    
    try:
        brain_input.connect()
        brain_output.connect()
        print("  ✓ Connected successfully!")
    except Exception as e:
        print(f"  ✗ Connection failed: {e}")
        print("\n⚠️  Make sure:")
        print("    1. FEAGI is running on localhost")
        print("    2. feagi_rust_py_libs is installed")
        return
    
    # === Main Agent Loop ===
    print("\n🤖 Starting agent loop...")
    print("  (Press Ctrl+C to stop)\n")
    
    try:
        loop_count = 0
        while True:
            loop_count += 1
            
            # === Simulate sensor readings ===
            # In real code, replace with actual hardware API calls
            
            # Generate fake camera frame (random noise)
            camera_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            
            # Simulate infrared reading (oscillating distance)
            ir_distance = 200.0 + 100.0 * np.sin(loop_count * 0.1)
            
            # === Update FEAGI inputs ===
            camera.set_frame(camera_frame)
            infrared_front.set_distance(ir_distance)
            
            # === Send to FEAGI ===
            brain_input.send()
            
            # === Receive from FEAGI ===
            brain_output.receive()
            
            # === Read motor commands ===
            head_angle = servo_head.get_angle()
            left_speed = motor_left.get_speed()
            right_speed = motor_right.get_speed()
            
            # === Apply to hardware ===
            # In real code, replace with actual hardware API calls
            # set_servo_angle(head_angle)
            # set_motor_speeds(left_speed, right_speed)
            
            # Print status every 10 loops
            if loop_count % 10 == 0:
                print(f"[Loop {loop_count:04d}] "
                      f"IR={ir_distance:6.1f}cm | "
                      f"Head={head_angle:5.1f}° | "
                      f"Motors=({left_speed:+.2f}, {right_speed:+.2f})")
            
            # Small delay
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping agent...")
    
    # === Cleanup ===
    brain_input.disconnect()
    brain_output.disconnect()
    print("✓ Disconnected")
    
    print("\n" + "=" * 60)
    print("Agent stopped successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()

