"""
Example 1: Basic Metrics Collection

Demonstrates how to collect basic metrics on agent performance.
"""

from feagi.pns.inputs import Camera
from feagi.pns.outputs import ServoMotor, RotaryMotor
from feagi.pns import brain_input, brain_output
from feagi.pns.observability import MetricsCollector

import numpy as np
import time


def main():
    print("=" * 60)
    print("Example 1: Basic Metrics Collection")
    print("=" * 60)
    
    # === Register devices ===
    print("\n📥 Registering devices...")
    camera = Camera.register(resolution=(640, 480))
    servo = ServoMotor.register(range=(0, 180))
    motor_left = RotaryMotor.register()
    motor_right = RotaryMotor.register()
    print("✓ Devices registered")
    
    # === Configure connection ===
    print("\n🔧 Configuring connection...")
    brain_input.configure(feagi_host="localhost")
    brain_output.configure(feagi_host="localhost")
    
    brain_input.connect()
    brain_output.connect()
    print("✓ Connected to FEAGI")
    
    # === Set up metrics collection ===
    print("\n📊 Setting up metrics collection...")
    metrics = MetricsCollector()
    
    # Attach to both input and output
    brain_input.attach_monitor(metrics)
    brain_output.attach_monitor(metrics)
    print("✓ Metrics collector attached")
    
    # === Run agent for 100 iterations ===
    print("\n🤖 Running agent (100 iterations)...")
    for i in range(100):
        # Generate fake camera frame
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        camera.set_frame(frame)
        
        # Send sensory data
        brain_input.send()
        
        # Receive motor commands
        brain_output.receive()
        
        # Read motor values (for demonstration)
        servo.get_angle()
        motor_left.get_speed()
        motor_right.get_speed()
        
        # Simulate processing time
        time.sleep(0.01)
        
        if (i + 1) % 20 == 0:
            print(f"  Processed {i + 1}/100 iterations...")
    
    print("✓ Agent run complete")
    
    # === Display metrics ===
    print("\n" + "=" * 60)
    print("METRICS SUMMARY")
    print("=" * 60)
    
    stats = metrics.get_statistics()
    
    print(f"\n📥 Sensory Input:")
    print(f"  Total packets sent:     {stats.input.total_packets}")
    print(f"  Total bytes sent:       {stats.input.total_bytes:,}")
    print(f"  Total neurons sent:     {stats.input.total_neurons:,}")
    print(f"  Avg packet size:        {stats.input.avg_packet_size:.2f} bytes")
    print(f"  Data rate:              {stats.input.data_rate_mbps:.2f} MB/s")
    print(f"  Packets/sec:            {stats.input.packets_per_sec:.2f}")
    
    print(f"\n📤 Motor Output:")
    print(f"  Total receives:         {stats.output.total_receives}")
    print(f"  Total commands:         {stats.output.total_commands}")
    print(f"  Avg latency:            {stats.output.avg_latency_ms:.2f} ms")
    
    print(f"\n⏱️  Uptime:")
    print(f"  Duration:               {stats.uptime_seconds:.2f} seconds")
    
    # === Export metrics ===
    print("\n💾 Exporting metrics...")
    metrics.export_json("metrics_basic.json")
    metrics.export_csv("metrics_basic.csv")
    print("✓ Metrics exported to:")
    print("  - metrics_basic.json")
    print("  - metrics_basic.csv")
    
    print("\n" + "=" * 60)
    print("✅ Example complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()

