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

import time
import argparse
from feagi.utils.system_monitor import start_system_monitoring, stop_system_monitoring, print_resource_report


def simulate_workload():
    """Simulate some CPU/memory intensive work."""
    print("🔥 Simulating CPU-intensive workload...")
    
    # CPU intensive task
    for i in range(3):
        result = sum(x * x for x in range(100000))
        print(f"   Iteration {i+1}: computed sum = {result}")
        time.sleep(1)
    
    # Memory intensive task
    print("💾 Simulating memory-intensive workload...")
    large_data = []
    for i in range(3):
        data_chunk = [x for x in range(50000)]
        large_data.append(data_chunk)
        print(f"   Allocated chunk {i+1}: {len(data_chunk)} integers")
        time.sleep(1)
    
    print("✅ Workload simulation complete")
    return len(large_data)


def main():
    parser = argparse.ArgumentParser(description="FEAGI System Resource Monitor Demo")
    parser.add_argument("--interval", type=float, default=2.0, help="Monitoring interval in seconds")
    parser.add_argument("--duration", type=int, default=15, help="Demo duration in seconds")
    parser.add_argument("--no-gpu", action="store_true", help="Disable GPU monitoring")
    parser.add_argument("--no-logging", action="store_true", help="Disable detailed logging")
    
    args = parser.parse_args()
    
    print("="*80)
    print("FEAGI SYSTEM RESOURCE MONITOR DEMO")
    print("="*80)
    print(f"⏱️  Monitoring interval: {args.interval}s")
    print(f"⏰ Demo duration: {args.duration}s")
    print(f"🎮 GPU monitoring: {'Disabled' if args.no_gpu else 'Enabled'}")
    print(f"📝 Detailed logging: {'Disabled' if args.no_logging else 'Enabled'}")
    print()
    
    try:
        # Start system monitoring
        print("🚀 Starting system resource monitor...")
        monitor = start_system_monitoring(
            monitoring_interval=args.interval,
            enable_gpu_monitoring=not args.no_gpu,
            enable_detailed_logging=not args.no_logging
        )
        
        if not monitor:
            print("❌ Failed to start system monitor")
            return 1
        
        print("✅ System monitor started successfully")
        print()
        
        # Wait a bit for initial measurements
        print("📊 Collecting baseline measurements...")
        time.sleep(args.interval * 2)
        
        # Run some workload simulation
        simulate_workload()
        
        # Wait for more measurements
        print(f"⏳ Collecting {args.duration - 8} more seconds of measurements...")
        remaining_time = max(1, args.duration - 8)  # Account for workload time
        time.sleep(remaining_time)
        
        # Print detailed report
        print("\n" + "="*80)
        print("FINAL RESOURCE USAGE REPORT")
        print("="*80)
        print_resource_report()
        
        # Show current usage
        current = monitor.get_current_usage()
        if current:
            print("📈 Current Usage Summary:")
            print(current.format_summary())
        
        # Show recent summary
        summary = monitor.get_usage_summary(last_n_entries=10)
        if summary:
            print("\n📊 Recent Performance Summary:")
            print(f"   Entries analyzed: {summary['entries_analyzed']}")
            print(f"   Time span: {summary['time_span_minutes']:.1f} minutes")
            print(f"   System CPU: avg {summary['system']['cpu_avg']:.1f}%, peak {summary['system']['cpu_peak']:.1f}%")
            print(f"   System Memory: avg {summary['system']['memory_mb_avg']:.1f} MB, peak {summary['system']['memory_mb_peak']:.1f} MB")
            print(f"   FEAGI Process CPU: avg {summary['process']['cpu_avg']:.1f}%, peak {summary['process']['cpu_peak']:.1f}%")
            print(f"   FEAGI Process Memory: avg {summary['process']['memory_mb_avg']:.1f} MB, peak {summary['process']['memory_mb_peak']:.1f} MB")
            
            if 'gpu' in summary:
                print(f"   GPU Performance:")
                for gpu in summary['gpu']:
                    print(f"      GPU {gpu['index']}: avg {gpu['utilization_avg']:.1f}%, peak {gpu['utilization_peak']:.1f}%")
        
        print()
        
    except KeyboardInterrupt:
        print("\n⚠️  Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        return 1
    finally:
        # Clean shutdown
        print("🛑 Stopping system monitor...")
        stop_system_monitoring()
        print("✅ Demo completed successfully")
    
    return 0


if __name__ == "__main__":
    exit(main()) 