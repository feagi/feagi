"""
Example 6: Quick Start - One-Liner Monitoring

Demonstrates the simplest way to enable monitoring.
"""

import os
from feagi.pns.inputs import Camera
from feagi.pns.outputs import ServoMotor
from feagi.pns import brain_input, brain_output
from feagi.pns.observability import enable_monitoring

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
    print("Example 6: Quick Start - One-Liner Monitoring")
    print("=" * 60)
    
    # === Register devices ===
    print("\n📥 Registering devices...")
    camera = Camera.register(resolution=(640, 480))
    servo = ServoMotor.register(range=(0, 180))
    print("✓ Devices registered")
    
    # === Configure connection ===
    print("\n🔧 Configuring connection...")
    agent_id = require_env("FEAGI_AGENT_DESCRIPTOR_B64")
    feagi_host = require_env("FEAGI_HOST")
    feagi_registration_port = parse_env_int("FEAGI_REGISTRATION_PORT")
    feagi_sensory_port = parse_env_int("FEAGI_SENSORY_PORT")
    feagi_motor_port = parse_env_int("FEAGI_MOTOR_PORT")
    feagi_connection_timeout_ms = parse_env_int("FEAGI_CONNECTION_TIMEOUT_MS")
    feagi_registration_retries = parse_env_int("FEAGI_REGISTRATION_RETRIES")
    feagi_heartbeat_interval_s = parse_env_float("FEAGI_HEARTBEAT_INTERVAL_S")

    brain_input.configure(feagi_host=feagi_host, feagi_port=feagi_sensory_port, transport="zmq")
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
    
    brain_input.connect()
    brain_output.connect()
    print("✓ Connected to FEAGI")
    
    # === Enable monitoring with one line ===
    print("\n🔍 Enabling monitoring (one-liner)...")
    
    metrics, logger = enable_monitoring(
        log_file="quick_start_data.log",
        log_level="INFO"
    )
    
    print("✓ Monitoring enabled!")
    print("  - Metrics collection: ON")
    print("  - Data logging: quick_start_data.log")
    
    # === Run agent ===
    print("\n🤖 Running agent (50 iterations)...")
    
    for i in range(50):
        # Generate frame
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        camera.set_frame(frame)
        
        # Send and receive
        brain_input.send()
        brain_output.receive()
        
        # Simulate processing
        time.sleep(0.02)
        
        if (i + 1) % 10 == 0:
            print(f"  Processed {i + 1}/50 iterations...")
    
    print("✓ Agent run complete")
    
    # === View results ===
    print("\n📊 Results:")
    metrics.print_summary()
    
    # === Cleanup ===
    logger.close()
    
    print("\n" + "=" * 60)
    print("✅ Quick start example complete!")
    print("=" * 60)
    
    print("\nThat's it! Just one line to enable monitoring:")
    print("  enable_monitoring(log_file='data.log')")
    print("\nYou get:")
    print("  ✓ Automatic metrics collection")
    print("  ✓ Structured data logging")
    print("  ✓ Easy-to-read summaries")
    print("  ✓ Exportable data files")


if __name__ == "__main__":
    main()

