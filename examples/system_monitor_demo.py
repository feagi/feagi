#!/usr/bin/env python3
"""
FEAGI System Resource Monitor Demo

This demonstrates how to use FEAGI's system resource monitoring capabilities
for tracking memory, CPU, and GPU usage during profiling mode using absolute
measurements that are comparable across different systems:
- CPU usage in equivalent cores (e.g., "1.2 cores")
- Memory usage in MB (not percentages)

Usage:
    python examples/system_monitor_demo.py

Note: To enable system monitoring in FEAGI, use the --profile flag:
    python -m feagi.main --profile
"""

import argparse
import time

from feagi.utils.system_monitor import (
    SystemMonitor,
    print_resource_report,
    start_system_monitoring,
    stop_system_monitoring,
)


def simulate_workload(workload_type="cpu"):
    """Simulate different types of workloads."""
    if workload_type == "cpu":
        print("Simulating CPU-intensive workload...")
        start_time = time.time()
        x = 0
        while time.time() - start_time < 2:  # 2 seconds of CPU work
            x += 1
    elif workload_type == "memory":
        print("Simulating memory-intensive workload...")
        # Allocate and use memory
        data = [0] * (10 * 1024 * 1024)  # 10M integers
        for i in range(len(data)):
            data[i] = i % 100
        time.sleep(1)
        del data

    print("Workload simulation complete")


def main():
    """Main demo function."""
    parser = argparse.ArgumentParser(description="System Resource Monitor Demo")
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Monitoring interval in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Demo duration in seconds (default: 30)",
    )
    parser.add_argument("--no-gpu", action="store_true", help="Disable GPU monitoring")
    parser.add_argument(
        "--simulate-workload", choices=["cpu", "memory"], help="Simulate workload type"
    )

    args = parser.parse_args()

    print("System Resource Monitor Demo")
    print("=" * 40)
    print(f"Monitoring interval: {args.interval}s")
    print(f"Demo duration: {args.duration}s")
    print(f"GPU monitoring: {'Disabled' if args.no_gpu else 'Enabled'}")
    print("=" * 40)

    # Create monitor instance
    monitor = SystemMonitor(enable_gpu=not args.no_gpu)

    print("Starting system resource monitor...")

    try:
        # Start monitoring
        monitor.start_monitoring()
    except Exception:
        print("Failed to start system monitor")
        return

    print("System monitor started successfully")

    try:
        print("Collecting baseline measurements...")

        # Let monitor collect some baseline data
        time.sleep(5)

        # Simulate workload if requested
        if args.simulate_workload:
            print(f"Running {args.simulate_workload} workload simulation...")
            simulate_workload(args.simulate_workload)

        print(f"Collecting {args.duration - 8} more seconds of measurements...")

        # Continue monitoring for the rest of the duration
        remaining_time = max(0, args.duration - 8)  # Account for startup time
        time.sleep(remaining_time)

        # Get current stats
        current = monitor.get_current_stats()
        averages = monitor.get_averages()
        peak = monitor.get_peaks()

        # Display results
        print("Current Usage Summary:")
        print(f"  CPU: {current.cpu_cores_used:.2f} cores")
        print(f"  Memory: {current.memory_mb:.1f} MB")
        print(
            f"  GPU Memory: {current.gpu_memory_mb if current.gpu_memory_mb is not None else 'N/A'} MB"
        )
        print(
            f"  GPU Utilization: {current.gpu_utilization if current.gpu_utilization is not None else 'N/A'}%"
        )

        print("\nRecent Performance Summary:")
        if averages:
            print(f"  Average CPU: {averages.cpu_cores_used:.2f} cores")
            print(f"  Average Memory: {averages.memory_mb:.1f} MB")
            print(
                f"  Average GPU Memory: {averages.gpu_memory_mb if averages.gpu_memory_mb is not None else 'N/A'} MB"
            )
            print(
                f"  Average GPU Utilization: {averages.gpu_utilization if averages.gpu_utilization is not None else 'N/A'}%"
            )

        if peak:
            print(f"  Peak CPU: {peak.cpu_cores_used:.2f} cores")
            print(f"  Peak Memory: {peak.memory_mb:.1f} MB")
            print(
                f"  Peak GPU Memory: {peak.gpu_memory_mb if peak.gpu_memory_mb is not None else 'N/A'} MB"
            )
            print(
                f"  Peak GPU Utilization: {peak.gpu_utilization if peak.gpu_utilization is not None else 'N/A'}%"
            )

    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"\nDemo failed: {e}")
    finally:
        # Stop monitoring
        monitor.stop_monitoring()
        print("Demo completed successfully")


if __name__ == "__main__":
    exit(main())
