"""
Example 3: Data Validation

Demonstrates how to validate data and detect anomalies.
"""

from feagi.pns.inputs import Camera
from feagi.pns import brain_input
from feagi.pns.observability import DataInspector

import numpy as np
import time


def main():
    print("=" * 60)
    print("Example 3: Data Validation")
    print("=" * 60)
    
    # === Register devices ===
    print("\nRegistering devices...")
    camera = Camera.register(resolution=(640, 480))
    print("Camera registered")
    
    # === Configure connection ===
    print("\nConfiguring connection...")
    brain_input.configure(feagi_host="localhost")
    brain_input.connect()
    print("Connected to FEAGI")
    
    # === Set up data inspection ===
    print("\n🔍 Setting up data validation...")
    inspector = DataInspector(
        validate_formats=True,
        check_ranges=True,
        detect_anomalies=True
    )
    
    brain_input.attach_monitor(inspector)
    print("Data inspector attached")
    
    # === Run agent with various data conditions ===
    print("\n🤖 Running agent with test scenarios...")
    
    scenarios = [
        ("Normal data", lambda: np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)),
        ("All zeros", lambda: np.zeros((480, 640, 3), dtype=np.uint8)),
        ("NaN values", lambda: np.full((480, 640, 3), np.nan, dtype=np.float32)),
        ("Normal data", lambda: np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)),
    ]
    
    for scenario_name, data_generator in scenarios:
        print(f"\n  Testing: {scenario_name}")
        
        for i in range(5):
            frame = data_generator()
            camera.set_frame(frame)
            brain_input.send()
            time.sleep(0.01)
        
        print("    Sent 5 packets")
    
    print("\nTest scenarios complete")
    
    # === Check validation report ===
    print("\n" + "=" * 60)
    print("VALIDATION REPORT")
    print("=" * 60)
    
    report = inspector.get_report()
    
    print(f"\nPackets inspected: {report.packets_inspected}")
    print(f"Errors:   {report.error_count}")
    print(f"Warnings: {report.warning_count}")
    print(f"Info:     {report.info_count}")
    
    if report.issues:
        print("\nIssues detected:")
        for issue in report.issues:
            print(f"  {issue}")
    else:
        print("\nNo issues detected.")
    
    # === Display detailed report ===
    print("\n" + "=" * 60)
    report.print_summary()
    
    # === Recommendations ===
    if report.has_errors():
        print("=" * 60)
        print("RECOMMENDATIONS")
        print("=" * 60)
        print("\nData validation errors detected. Common fixes:")
        print("  1. Check sensor connections")
        print("  2. Verify data format correctness")
        print("  3. Handle edge cases (missing data, errors)")
        print("  4. Add input validation before sending to FEAGI")
    
    print("\n" + "=" * 60)
    print("Example complete.")
    print("=" * 60)


if __name__ == "__main__":
    main()

