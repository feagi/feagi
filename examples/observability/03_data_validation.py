"""
Example 3: Data Validation

Demonstrates how to validate data and detect anomalies.
"""

import os
from feagi.pns.inputs import Camera
from feagi.pns import brain_input
from feagi.pns.observability import DataInspector

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
    print("Example 3: Data Validation")
    print("=" * 60)
    
    # === Register devices ===
    print("\nRegistering devices...")
    camera = Camera.register(resolution=(640, 480))
    print("Camera registered")
    
    # === Configure connection ===
    print("\nConfiguring connection...")
    feagi_host = require_env("FEAGI_HOST")
    feagi_sensory_port = parse_env_int("FEAGI_SENSORY_PORT")
    feagi_motor_port = parse_env_int("FEAGI_MOTOR_PORT")
    feagi_api_port = parse_env_int("FEAGI_API_PORT")
    feagi_http_timeout_s = parse_env_float("FEAGI_HTTP_TIMEOUT_S")
    feagi_heartbeat_interval_s = parse_env_float("FEAGI_HEARTBEAT_INTERVAL_S")
    feagi_registration_port = parse_env_int("FEAGI_REGISTRATION_PORT")
    feagi_connection_timeout_ms = parse_env_int("FEAGI_CONNECTION_TIMEOUT_MS")
    feagi_registration_retries = parse_env_int("FEAGI_REGISTRATION_RETRIES")
    feagi_auth_token_b64 = require_env("FEAGI_AUTH_TOKEN_B64")

    brain_input.configure(
        feagi_host=feagi_host,
        feagi_port=feagi_sensory_port,
        motor_port=feagi_motor_port,
        transport="zmq",
        api_port=feagi_api_port,
        feagi_http_timeout_s=feagi_http_timeout_s,
        heartbeat_interval_s=feagi_heartbeat_interval_s,
        heartbeat_join_timeout_s=2.0,
        registration_port=feagi_registration_port,
        connection_timeout_ms=feagi_connection_timeout_ms,
        registration_retries=feagi_registration_retries,
        auth_token_b64=feagi_auth_token_b64,
    )
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

