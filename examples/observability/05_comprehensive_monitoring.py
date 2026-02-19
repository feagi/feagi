"""
Example 5: Comprehensive Monitoring

Demonstrates using all observability features together.
"""

import os
from feagi.pns.inputs import Camera
from feagi.pns.outputs import ServoMotor, RotaryMotor
from feagi.pns import brain_input, brain_output
from feagi.pns.observability import (
    MetricsCollector,
    DataLogger,
    DataInspector,
    Profiler
)

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
    print("Example 5: Comprehensive Monitoring")
    print("=" * 60)
    
    # === Register devices ===
    print("\nRegistering devices...")
    camera = Camera.register(resolution=(640, 480))
    servo = ServoMotor.register(range=(0, 180))
    motor_left = RotaryMotor.register()
    motor_right = RotaryMotor.register()
    print("Devices registered")
    
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
    print("Connected to FEAGI")
    
    # === Set up all monitors ===
    print("\n🔍 Setting up comprehensive monitoring...")
    
    # Metrics collection
    metrics = MetricsCollector(log_level="INFO")
    print("  MetricsCollector created")
    
    # Data logging (10% sampling for lower overhead)
    logger = DataLogger(
        output_file="comprehensive_data.jsonl",
        format="jsonl",
        log_inputs=True,
        log_outputs=True,
        sample_rate=0.1  # Log 10% of packets
    )
    print("  DataLogger created (10% sampling)")
    
    # Data validation
    inspector = DataInspector(
        validate_formats=True,
        check_ranges=True,
        detect_anomalies=True
    )
    print("  DataInspector created")
    
    # Performance profiling
    profiler = Profiler()
    print("  Profiler created")
    
    # Attach all monitors
    monitors = [metrics, logger, inspector, profiler]
    
    for monitor in monitors:
        brain_input.attach_monitor(monitor)
        brain_output.attach_monitor(monitor)
    
    print("All monitors attached")
    
    # === Run agent ===
    print("\n🤖 Running monitored agent (100 iterations)...")
    
    for i in range(100):
        # Generate frame
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        camera.set_frame(frame)
        
        # Send sensory data
        brain_input.send()
        
        # Receive motor commands
        brain_output.receive()
        
        # Read motor values
        servo.get_angle()
        motor_left.get_speed()
        motor_right.get_speed()
        
        # Simulate processing
        time.sleep(0.01)
        
        if (i + 1) % 25 == 0:
            print(f"  Processed {i + 1}/100 iterations...")
    
    print("Agent run complete")
    
    # === Close logger ===
    print("\n💾 Closing data logger...")
    logger.close()
    print("Logger closed")
    
    # === Display all results ===
    print("\n" + "=" * 60)
    print("📊 METRICS SUMMARY")
    print("=" * 60)
    metrics.print_summary()
    
    print("=" * 60)
    print("🔍 VALIDATION REPORT")
    print("=" * 60)
    validation_report = inspector.get_report()
    
    if validation_report.has_errors() or validation_report.has_warnings():
        validation_report.print_summary()
    else:
        print("\nNo validation issues detected.")
        print(f"   Inspected {validation_report.packets_inspected} packets")
    
    print("\n" + "=" * 60)
    print("⚡ PERFORMANCE PROFILE")
    print("=" * 60)
    profile = profiler.get_profile()
    profile.print_summary()
    
    # === Export data ===
    print("=" * 60)
    print("💾 EXPORTING DATA")
    print("=" * 60)
    
    metrics.export_json("comprehensive_metrics.json")
    metrics.export_csv("comprehensive_metrics.csv")
    
    print("\nExported:")
    print("  - comprehensive_metrics.json")
    print("  - comprehensive_metrics.csv")
    print("  - comprehensive_data.jsonl")
    
    # === Final summary ===
    print("\n" + "=" * 60)
    print("📋 MONITORING SUMMARY")
    print("=" * 60)
    
    stats = metrics.get_statistics()
    
    print(f"\nData Flow:")
    print(f"  Packets sent:       {stats.input.total_packets}")
    print(f"  Data rate:          {stats.input.data_rate_mbps:.2f} MB/s")
    print(f"  Commands received:  {stats.output.total_commands}")
    
    print(f"\nQuality:")
    print(f"  Validation errors:  {validation_report.error_count}")
    print(f"  Validation warnings:{validation_report.warning_count}")
    
    bottlenecks = profile.get_bottlenecks(threshold_ms=10.0)
    print(f"\nPerformance:")
    print(f"  Bottlenecks (>10ms):{len(bottlenecks)}")
    
    if bottlenecks:
        for op in bottlenecks:
            print(f"    - {op.operation_name}: {op.avg_time_ms:.2f} ms")
    
    print(f"\nData Logged:")
    print(f"  Sample rate:        10%")
    print(f"  Format:             JSONL")
    print(f"  File:               comprehensive_data.jsonl")
    
    print("\n" + "=" * 60)
    print("Comprehensive monitoring example complete.")
    print("=" * 60)
    
    print("\nYou now have:")
    print("  1. Performance metrics (JSON/CSV)")
    print("  2. Validation report (shown above)")
    print("  3. Performance profile (shown above)")
    print("  4. Detailed packet logs (JSONL)")
    print("\nUse these for:")
    print("  - Debugging issues")
    print("  - Performance optimization")
    print("  - Regression testing")
    print("  - Production monitoring")


if __name__ == "__main__":
    main()

