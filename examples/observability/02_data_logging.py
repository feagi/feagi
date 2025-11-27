"""
Example 2: Data Logging

Demonstrates how to log sensory and motor data in structured formats.
"""

from feagi.pns.inputs import Camera
from feagi.pns.outputs import ServoMotor
from feagi.pns import brain_input, brain_output
from feagi.pns.observability import DataLogger

import numpy as np
import time


def main():
    print("=" * 60)
    print("Example 2: Data Logging")
    print("=" * 60)
    
    # === Register devices ===
    print("\n📥 Registering devices...")
    camera = Camera.register(resolution=(320, 240))
    servo = ServoMotor.register(range=(0, 180))
    print("✓ Devices registered")
    
    # === Configure connection ===
    print("\n🔧 Configuring connection...")
    brain_input.configure(feagi_host="localhost")
    brain_output.configure(feagi_host="localhost")
    
    brain_input.connect()
    brain_output.connect()
    print("✓ Connected to FEAGI")
    
    # === Set up data logging ===
    print("\n📝 Setting up data logging...")
    
    # JSONL logger (recommended - streams to disk)
    logger_jsonl = DataLogger(
        output_file="data_log.jsonl",
        format="jsonl",
        log_inputs=True,
        log_outputs=True,
        sample_rate=1.0  # Log 100% of packets
    )
    
    # CSV logger (for spreadsheet analysis)
    logger_csv = DataLogger(
        output_file="data_log.csv",
        format="csv",
        log_inputs=True,
        log_outputs=True,
        sample_rate=1.0
    )
    
    # Attach loggers
    brain_input.attach_monitor(logger_jsonl)
    brain_input.attach_monitor(logger_csv)
    brain_output.attach_monitor(logger_jsonl)
    brain_output.attach_monitor(logger_csv)
    
    print("✓ Loggers attached:")
    print("  - JSONL: data_log.jsonl")
    print("  - CSV:   data_log.csv")
    
    # === Run agent ===
    print("\n🤖 Running agent (50 iterations)...")
    for i in range(50):
        # Generate fake camera frame
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        camera.set_frame(frame)
        
        # Send sensory data
        brain_input.send()
        
        # Receive motor commands
        brain_output.receive()
        
        # Simulate processing
        time.sleep(0.02)
        
        if (i + 1) % 10 == 0:
            print(f"  Logged {i + 1}/50 packets...")
    
    print("✓ Agent run complete")
    
    # === Close loggers ===
    print("\n💾 Closing loggers...")
    logger_jsonl.close()
    logger_csv.close()
    print("✓ Loggers closed and data flushed")
    
    # === Display sample data ===
    print("\n" + "=" * 60)
    print("SAMPLE LOG DATA (JSONL)")
    print("=" * 60)
    
    print("\nFirst 3 entries from data_log.jsonl:")
    try:
        with open("data_log.jsonl", 'r') as f:
            for i, line in enumerate(f):
                if i < 3:
                    print(f"  {line.strip()}")
                else:
                    break
    except FileNotFoundError:
        print("  (Log file not created - FEAGI may not be running)")
    
    print("\n" + "=" * 60)
    print("✅ Example complete!")
    print("=" * 60)
    print("\nLog files created:")
    print("  - data_log.jsonl (JSON Lines format)")
    print("  - data_log.csv (CSV format)")
    print("\nYou can analyze these logs with standard tools:")
    print("  - jq: cat data_log.jsonl | jq '.'")
    print("  - Excel: Open data_log.csv")
    print("  - Python pandas: pd.read_json('data_log.jsonl', lines=True)")


if __name__ == "__main__":
    main()

