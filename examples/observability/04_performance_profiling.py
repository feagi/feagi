"""
Example 4: Performance Profiling

Demonstrates how to profile agent performance and identify bottlenecks.
"""

import os
from feagi.pns.inputs import Camera
from feagi.pns.outputs import ServoMotor, RotaryMotor
from feagi.pns import brain_input, brain_output
from feagi.pns.observability import Profiler

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
    print("Example 4: Performance Profiling")
    print("=" * 60)
    
    # === Register devices ===
    print("\nRegistering devices...")
    camera = Camera.register(resolution=(1920, 1080))  # High resolution
    servo = ServoMotor.register(range=(0, 180))
    motor_left = RotaryMotor.register()
    motor_right = RotaryMotor.register()
    print("Devices registered (high-res camera for profiling)")
    
    # === Configure connection ===
    print("\nConfiguring connection...")
    agent_id = require_env("FEAGI_AGENT_DESCRIPTOR_B64")
    feagi_host = require_env("FEAGI_HOST")
    feagi_registration_port = parse_env_int("FEAGI_REGISTRATION_PORT")
    feagi_sensory_port = parse_env_int("FEAGI_SENSORY_PORT")
    feagi_motor_port = parse_env_int("FEAGI_MOTOR_PORT")
    feagi_connection_timeout_ms = parse_env_int("FEAGI_CONNECTION_TIMEOUT_MS")
    feagi_registration_retries = parse_env_int("FEAGI_REGISTRATION_RETRIES")
    feagi_heartbeat_interval_s = parse_env_float("FEAGI_HEARTBEAT_INTERVAL_S")
    feagi_api_port = parse_env_int("FEAGI_API_PORT")
    feagi_http_timeout_s = parse_env_float("FEAGI_HTTP_TIMEOUT_S")

    brain_input.configure(
        feagi_host=feagi_host,
        feagi_port=feagi_sensory_port,
        motor_port=feagi_motor_port,
        transport="zmq",
        api_port=feagi_api_port,
        feagi_http_timeout_s=feagi_http_timeout_s,
        heartbeat_interval_s=feagi_heartbeat_interval_s,
        heartbeat_join_timeout_s=2.0,
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
    
    brain_input.connect()
    brain_output.connect()
    print("Connected to FEAGI")
    
    # === Set up profiler ===
    print("\n⚡ Setting up performance profiler...")
    profiler = Profiler()
    
    brain_input.attach_monitor(profiler)
    brain_output.attach_monitor(profiler)
    print("Profiler attached")
    
    # === Run performance benchmark ===
    print("\n🤖 Running performance benchmark (200 iterations)...")
    start_time = time.time()
    
    for i in range(200):
        # Generate high-res frame
        frame = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)
        camera.set_frame(frame)
        
        # Send sensory data
        brain_input.send()
        
        # Receive motor commands
        brain_output.receive()
        
        # Read motor values
        servo.get_angle()
        motor_left.get_speed()
        motor_right.get_speed()
        
        # No artificial delay - measure real performance
        
        if (i + 1) % 50 == 0:
            print(f"  Completed {i + 1}/200 iterations...")
    
    total_time = time.time() - start_time
    print(f"Benchmark complete in {total_time:.2f} seconds")
    print(f"  Average: {total_time / 200 * 1000:.2f} ms per iteration")
    
    # === Display performance profile ===
    print("\n" + "=" * 60)
    print("PERFORMANCE PROFILE")
    print("=" * 60)
    
    profile = profiler.get_profile()
    profile.print_summary()
    
    # === Identify bottlenecks ===
    print("=" * 60)
    print("BOTTLENECK ANALYSIS")
    print("=" * 60)
    
    bottlenecks = profile.get_bottlenecks(threshold_ms=5.0)
    
    if bottlenecks:
        print(f"\n⚠️  Found {len(bottlenecks)} bottleneck(s) (>5ms):")
        for op in bottlenecks:
            print(f"\n  Operation: {op.operation_name}")
            print(f"    Avg time:  {op.avg_time_ms:.2f} ms")
            print(f"    Min time:  {op.min_time_ms:.2f} ms")
            print(f"    Max time:  {op.max_time_ms:.2f} ms")
            print(f"    Calls:     {op.call_count}")
            
            # Provide optimization suggestions
            print("    💡 Suggestions:")
            if op.operation_name == "send":
                print("       - Reduce camera resolution")
                print("       - Use delta encoding instead of absolute")
                print("       - Enable compression")
            elif op.operation_name == "encoding":
                print("       - Lower image resolution")
                print("       - Reduce color depth")
                print("       - Use Rust-backed encoding (already used)")
            elif op.operation_name == "transmission":
                print("       - Check network latency")
                print("       - Use local FEAGI instance")
                print("       - Verify ZMQ configuration")
            elif op.operation_name == "receive":
                print("       - Reduce motor sampling rate")
                print("       - Optimize motor command processing")
    else:
        print("\nNo significant bottlenecks detected.")
        print("   All operations running efficiently (<5ms average)")
    
    # === Performance summary ===
    print("\n" + "=" * 60)
    print("PERFORMANCE SUMMARY")
    print("=" * 60)
    
    print(f"\nThroughput:")
    print(f"  Iterations/sec:  {200 / total_time:.2f}")
    print(f"  Avg cycle time:  {total_time / 200 * 1000:.2f} ms")
    
    if profile.encoding_time_ms > 0:
        print(f"\nOperation breakdown:")
        print(f"  Encoding:        {profile.encoding_time_ms:.2f} ms")
        print(f"  Serialization:   {profile.serialization_time_ms:.2f} ms")
        print(f"  Transmission:    {profile.transmission_time_ms:.2f} ms")
    
    print("\n" + "=" * 60)
    print("Example complete.")
    print("=" * 60)


if __name__ == "__main__":
    main()

