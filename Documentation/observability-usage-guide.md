# FEAGI PNS Observability - Comprehensive Usage Guide

**Version:** 1.0  
**Date:** November 27, 2025  
**Module:** `feagi.pns.observability`

---

## Table of Contents

1. [Introduction](#introduction)
2. [Quick Start](#quick-start)
3. [Core Concepts](#core-concepts)
4. [Components](#components)
   - [MetricsCollector](#metricscollector)
   - [DataLogger](#datalogger)
   - [DataInspector](#datainspector)
   - [Profiler](#profiler)
5. [Common Use Cases](#common-use-cases)
6. [Advanced Usage](#advanced-usage)
7. [API Reference](#api-reference)
8. [Performance Considerations](#performance-considerations)
9. [Troubleshooting](#troubleshooting)
10. [Best Practices](#best-practices)

---

## Introduction

The FEAGI PNS Observability module provides developers with powerful tools for monitoring, debugging, and optimizing FEAGI agents. It offers real-time visibility into sensory data flows, motor command processing, and overall agent performance.

### Key Features

- **📊 Metrics Collection**: Track data rates, packet sizes, latencies, and throughput
- **📝 Structured Logging**: Log packets in JSON, JSONL, or CSV formats
- **🔍 Data Validation**: Detect format errors, anomalies, and range violations
- **⚡ Performance Profiling**: Identify bottlenecks and optimize performance
- **🔌 Zero-Overhead**: Minimal impact when enabled, zero when disabled
- **🎯 Easy Integration**: Simple API with one-liner setup

### When to Use Observability

- **Development**: Debug sensor/motor issues and validate data flows
- **Testing**: Regression testing with metrics-based validation
- **Optimization**: Profile performance and identify bottlenecks
- **Production**: Monitor deployed agents in real-time
- **Learning**: Understand how FEAGI agents process data

---

## Quick Start

### One-Liner Setup

```python
from feagi.pns.observability import enable_monitoring

# Enable monitoring with defaults
metrics, logger = enable_monitoring(
    log_file="agent_monitor.log",
    log_level="INFO"
)

# Your agent code here...
# camera.set_frame(frame)
# brain_input.send()
# brain_output.receive()

# View results
metrics.print_summary()
logger.close()
```

### Using Context Manager

```python
from feagi.pns.observability import monitor_session

with monitor_session(log_file="debug_session.log") as session:
    # Run agent code
    for i in range(100):
        camera.set_frame(frame)
        brain_input.send()
        brain_output.receive()
    
    # Metrics automatically saved on exit

print(f"Processed {session.packets} packets")
```

---

## Core Concepts

### Observer Pattern

Observability uses the Observer pattern - monitors "attach" to `brain_input` and `brain_output` to receive notifications about data operations.

```
┌─────────────┐         ┌──────────────┐
│ brain_input │────────>│   Monitor 1  │
│             │         │ (Metrics)    │
│             │    ┌───>│   Monitor 2  │
│             │────┤    │ (Logger)     │
│             │    └───>│   Monitor 3  │
│             │         │ (Inspector)  │
└─────────────┘         └──────────────┘
```

### Monitor Lifecycle

1. **Attach**: Monitor registers with brain_input/brain_output
2. **Observe**: Monitor receives callbacks on data operations
3. **Process**: Monitor collects metrics, logs data, validates, etc.
4. **Report**: Monitor provides results via API calls
5. **Detach**: Monitor can be removed when no longer needed

### Data Flow

```
Agent Code           Brain Input/Output       Monitors
    │                        │                    │
    ├──> send() ────────────>│                    │
    │                        ├──> on_send_start ─>│
    │                        │                    ├──> Record metrics
    │                        │                    ├──> Log data
    │                        │                    └──> Validate
    │                        │<── on_send_complete ┤
    │<───────────────────────┤                    │
```

---

## Components

### MetricsCollector

Collects statistics on data flow through brain_input and brain_output.

#### Features

- Packet counts and sizes
- Neuron counts
- Data rates (MB/s, packets/sec)
- Latencies and durations
- Export to JSON/CSV

#### Basic Usage

```python
from feagi.pns.observability import MetricsCollector

# Create collector
metrics = MetricsCollector()

# Attach to monitors
brain_input.attach_monitor(metrics)
brain_output.attach_monitor(metrics)

# Run agent...

# Get statistics
stats = metrics.get_statistics()

print(f"Packets sent: {stats.input.total_packets}")
print(f"Data rate: {stats.input.data_rate_mbps:.2f} MB/s")
print(f"Avg latency: {stats.output.avg_latency_ms:.2f} ms")
```

#### Print Summary

```python
metrics.print_summary()
```

Output:
```
============================================================
FEAGI Agent Metrics Summary
============================================================

📊 Uptime: 60.45 seconds

📥 Sensory Input:
  Total packets sent:     1500
  Total bytes sent:       12,345,678
  Total neurons sent:     3,086,400
  Avg packet size:        8,230.45 bytes
  Avg neurons/packet:     2,057.60
  Data rate:              2.04 MB/s
  Packets/sec:            24.81
  Avg send duration:      1.23 ms

📤 Motor Output:
  Total receives:         1500
  Total commands:         750
  Avg commands/receive:   0.50
  Avg latency:            0.85 ms
  Commands/sec:           12.40

============================================================
```

#### Export Metrics

```python
# Export as JSON
metrics.export_json("metrics.json")

# Export as CSV
metrics.export_csv("metrics.csv")
```

#### Reset Statistics

```python
metrics.reset()  # Clear all accumulated data
```

---

### DataLogger

Logs sensory input and motor output data in structured formats.

#### Features

- Multiple formats: JSON, JSONL (JSON Lines), CSV
- Configurable sampling rates
- Optional data samples in logs
- Automatic flush and close

#### Basic Usage

```python
from feagi.pns.observability import DataLogger

# Create logger
logger = DataLogger(
    output_file="agent_data.jsonl",
    format="jsonl",
    log_inputs=True,
    log_outputs=True,
    sample_rate=1.0  # Log 100% of packets
)

# Attach to monitors
brain_input.attach_monitor(logger)
brain_output.attach_monitor(logger)

# Run agent...

# Close logger (flushes data)
logger.close()
```

#### Sampling

Log only 10% of packets for reduced overhead:

```python
logger = DataLogger(
    output_file="sampled_data.jsonl",
    format="jsonl",
    sample_rate=0.1  # Log 10% of packets
)
```

#### Include Data Samples

Include actual data values in logs:

```python
logger = DataLogger(
    output_file="detailed_data.jsonl",
    format="jsonl",
    include_data_samples=True,
    max_sample_size=10  # Include first 10 data points
)
```

#### Output Formats

**JSONL (JSON Lines)** - One JSON object per line (recommended):
```json
{"timestamp":"2025-11-27T10:30:45.123Z","packet_id":1,"type":"sensory_input","neuron_count":307200,"packet_size_bytes":1228800,"duration_ms":1.23,"cortical_areas":["camera_0"]}
{"timestamp":"2025-11-27T10:30:45.140Z","packet_id":2,"type":"motor_output","command_count":2,"duration_ms":0.85}
```

**JSON** - Single array (loaded in memory):
```json
[
  {
    "timestamp": "2025-11-27T10:30:45.123Z",
    "packet_id": 1,
    "type": "sensory_input",
    "neuron_count": 307200,
    "packet_size_bytes": 1228800,
    "duration_ms": 1.23,
    "cortical_areas": ["camera_0"]
  }
]
```

**CSV** - Tabular format:
```csv
timestamp,type,cortical_area,neuron_count,packet_size_bytes,duration_ms,command_count
2025-11-27T10:30:45.123Z,sensory_input,camera_0,307200,1228800,1.23,0
2025-11-27T10:30:45.140Z,motor_output,,0,0,0.85,2
```

---

### DataInspector

Validates data formats and detects anomalies.

#### Features

- Format validation
- Range checking
- Anomaly detection (zeros, NaN, outliers)
- Severity levels (INFO, WARNING, ERROR)
- Detailed validation reports

#### Basic Usage

```python
from feagi.pns.observability import DataInspector

# Create inspector
inspector = DataInspector(
    validate_formats=True,
    check_ranges=True,
    detect_anomalies=True
)

# Attach to monitor
brain_input.attach_monitor(inspector)

# Run agent...

# Get validation report
report = inspector.get_report()

if report.has_errors():
    print(f"Found {report.error_count} errors!")
    report.print_summary()
```

#### Validation Report

```python
report = inspector.get_report()

# Check for issues
if report.has_errors():
    print(f"Errors: {report.error_count}")
if report.has_warnings():
    print(f"Warnings: {report.warning_count}")

# Print detailed report
report.print_summary()
```

Output:
```
============================================================
Data Validation Report
============================================================
Packets inspected: 1500
Errors:   2
Warnings: 5
Info:     10

Issues:
  [ERROR] Packet #45: Packet size is 0 but neuron count is 1024
  [ERROR] Packet #103 in camera_0: Data sample contains NaN values
  [WARNING] Packet #200: Slow send operation: 125.34 ms
  [WARNING] Packet #500: Very large packet: 105.32 MB
  [WARNING] Packet #750: Data sample contains all zeros
  [INFO] Packet #1000: No motor commands received
============================================================
```

#### Validation Categories

**Format Validation:**
- Empty packets
- Size inconsistencies
- Missing cortical areas

**Range Validation:**
- Unusually large packets (>100MB)
- Slow operations
- Out-of-bound values

**Anomaly Detection:**
- All-zero data
- NaN values
- Unexpected patterns

---

### Profiler

Profiles performance to identify bottlenecks.

#### Features

- Operation timing
- Min/max/average durations
- Bottleneck detection
- Performance reports

#### Basic Usage

```python
from feagi.pns.observability import Profiler

# Create profiler
profiler = Profiler()

# Attach to monitors
brain_input.attach_monitor(profiler)
brain_output.attach_monitor(profiler)

# Run agent...

# Get profile
profile = profiler.get_profile()

# Print summary
profile.print_summary()
```

#### Performance Report

```python
profile.print_summary()
```

Output:
```
============================================================
Performance Profile
============================================================

Operation                Calls    Avg (ms)    Min (ms)    Max (ms)
----------------------------------------------------------------------
send                      1500        1.23        0.85        5.67
encoding                  1500        0.45        0.32        2.10
serialization             1500        0.32        0.25        1.45
transmission              1500        0.46        0.28        1.98
receive                   1500        0.85        0.55        3.20

⚠️  Bottlenecks (>10ms average):
  - None detected

============================================================
```

#### Identify Bottlenecks

```python
# Get operations slower than threshold
bottlenecks = profile.get_bottlenecks(threshold_ms=10.0)

for op in bottlenecks:
    print(f"Slow: {op.operation_name} - {op.avg_time_ms:.2f} ms")
    print(f"  Calls: {op.call_count}")
    print(f"  Max: {op.max_time_ms:.2f} ms")
```

#### Track Specific Operations

```python
profiler = Profiler(
    track_operations=["encoding", "transmission"]
)
```

---

## Common Use Cases

### Use Case 1: Debugging Camera Input

**Problem**: Camera data not reaching FEAGI correctly.

**Solution**: Use DataInspector and DataLogger to diagnose:

```python
from feagi.pns.observability import DataInspector, DataLogger

# Enable detailed logging
inspector = DataInspector(
    validate_formats=True,
    check_ranges=True,
    detect_anomalies=True
)

logger = DataLogger(
    output_file="camera_debug.jsonl",
    format="jsonl",
    include_data_samples=True,
    max_sample_size=10
)

brain_input.attach_monitor(inspector)
brain_input.attach_monitor(logger)

# Run agent
for i in range(10):
    camera.set_frame(frame)
    brain_input.send()

# Check for issues
report = inspector.get_report()
if report.has_errors():
    print("❌ Found errors in camera data:")
    for issue in report.issues:
        print(f"  {issue}")

logger.close()
```

### Use Case 2: Performance Optimization

**Problem**: Agent running slower than expected.

**Solution**: Use Profiler to identify bottlenecks:

```python
from feagi.pns.observability import Profiler

# Profile performance
profiler = Profiler()
brain_input.attach_monitor(profiler)

# Run benchmark
for i in range(1000):
    camera.set_frame(frame)
    brain_input.send()

# Analyze results
profile = profiler.get_profile()
profile.print_summary()

# Check for slow operations
bottlenecks = profile.get_bottlenecks(threshold_ms=5.0)
if bottlenecks:
    print("\n⚠️ Performance Issues:")
    for op in bottlenecks:
        print(f"  {op.operation_name}: {op.avg_time_ms:.2f} ms")
        
        # Suggest optimizations
        if op.operation_name == "encoding":
            print("    💡 Try: Reduce resolution or use delta encoding")
        elif op.operation_name == "transmission":
            print("    💡 Try: Check network latency")
```

### Use Case 3: Regression Testing

**Problem**: Need to validate performance doesn't degrade.

**Solution**: Use MetricsCollector with assertions:

```python
from feagi.pns.observability import MetricsCollector

# Collect baseline metrics
metrics = MetricsCollector()
brain_input.attach_monitor(metrics)
brain_output.attach_monitor(metrics)

# Run standard test scenario
run_standard_test()

# Validate against baseline
stats = metrics.get_statistics()

# Assert performance requirements
assert stats.input.data_rate_mbps >= 2.0, \
    f"Data rate below baseline: {stats.input.data_rate_mbps:.2f} MB/s"

assert stats.output.avg_latency_ms <= 5.0, \
    f"Latency too high: {stats.output.avg_latency_ms:.2f} ms"

assert stats.input.packets_per_sec >= 20.0, \
    f"Packet rate too low: {stats.input.packets_per_sec:.2f} packets/sec"

# Export for CI/CD
metrics.export_json("test_metrics.json")

print("✅ All performance tests passed!")
```

### Use Case 4: Development Monitoring

**Problem**: Need visibility during development.

**Solution**: Enable comprehensive monitoring:

```python
from feagi.pns.observability import (
    MetricsCollector,
    DataLogger,
    DataInspector,
    Profiler
)

# Create all monitors
metrics = MetricsCollector()
logger = DataLogger(output_file="dev_session.jsonl", format="jsonl")
inspector = DataInspector()
profiler = Profiler()

# Attach to both input and output
for monitor in [metrics, logger, inspector, profiler]:
    brain_input.attach_monitor(monitor)
    brain_output.attach_monitor(monitor)

# Run development session
try:
    while True:
        camera.set_frame(get_camera_frame())
        infrared.set_distance(get_distance())
        brain_input.send()
        
        brain_output.receive()
        motor_left.get_speed()
        motor_right.get_speed()
        
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\nDevelopment session ended")

# View results
print("\n" + "=" * 60)
print("Session Summary")
print("=" * 60)

metrics.print_summary()

validation_report = inspector.get_report()
if validation_report.has_errors():
    print("\n⚠️ Validation Issues Found:")
    validation_report.print_summary()

profile = profiler.get_profile()
print("\nPerformance Profile:")
profile.print_summary()

logger.close()
```

### Use Case 5: Production Monitoring

**Problem**: Monitor deployed agents in real-time.

**Solution**: Lightweight metrics with periodic export:

```python
from feagi.pns.observability import MetricsCollector
import threading
import time

# Create metrics collector
metrics = MetricsCollector()
brain_input.attach_monitor(metrics)
brain_output.attach_monitor(metrics)

# Periodic export thread
def export_metrics_periodically():
    while True:
        time.sleep(300)  # Every 5 minutes
        try:
            metrics.export_json(f"metrics_{int(time.time())}.json")
            print("✅ Metrics exported")
        except Exception as e:
            print(f"❌ Failed to export metrics: {e}")

export_thread = threading.Thread(target=export_metrics_periodically, daemon=True)
export_thread.start()

# Run agent
while True:
    # Agent code...
    pass
```

---

## Advanced Usage

### Multiple Monitors

Combine multiple monitors for comprehensive observability:

```python
# Create monitors
metrics = MetricsCollector(log_level="INFO")
logger = DataLogger(output_file="data.jsonl", format="jsonl", sample_rate=0.1)
inspector = DataInspector(detect_anomalies=True)
profiler = Profiler()

# Attach all monitors
monitors = [metrics, logger, inspector, profiler]

for monitor in monitors:
    brain_input.attach_monitor(monitor)
    brain_output.attach_monitor(monitor)
```

### Conditional Monitoring

Enable monitoring based on conditions:

```python
import os

# Only enable in debug mode
if os.getenv("DEBUG", "false").lower() == "true":
    metrics = MetricsCollector()
    brain_input.attach_monitor(metrics)
    print("🔍 Debug monitoring enabled")
```

### Custom Monitor

Create a custom monitor by extending the base class:

```python
from feagi.pns.observability.monitor import Monitor
from typing import Dict, Any

class CustomMonitor(Monitor):
    def __init__(self):
        super().__init__()
        self.alert_threshold = 1000  # Alert if >1000 neurons
    
    def on_send_start(self, data: Dict[str, Any]):
        pass
    
    def on_send_complete(self, data: Dict[str, Any]):
        neuron_count = data.get('neuron_count', 0)
        if neuron_count > self.alert_threshold:
            print(f"⚠️ Alert: Large packet sent ({neuron_count} neurons)")
    
    def on_receive_start(self, data: Dict[str, Any]):
        pass
    
    def on_receive_complete(self, data: Dict[str, Any]):
        pass

# Use custom monitor
custom = CustomMonitor()
brain_input.attach_monitor(custom)
```

### Dynamic Enable/Disable

Enable or disable monitoring at runtime:

```python
metrics = MetricsCollector()
brain_input.attach_monitor(metrics)

# Disable temporarily
metrics.disable()

# Run agent without monitoring overhead
for i in range(1000):
    brain_input.send()

# Re-enable
metrics.enable()
```

### Export with Timestamps

Include timestamps in exported filenames:

```python
from datetime import datetime

# Export with timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
metrics.export_json(f"metrics_{timestamp}.json")
logger = DataLogger(output_file=f"data_{timestamp}.jsonl", format="jsonl")
```

---

## API Reference

### Monitor Base Class

```python
class Monitor(ABC):
    def __init__(self, enabled: bool = True, log_level: str = "INFO")
    def enable()
    def disable()
    def on_send_start(data: Dict[str, Any])
    def on_send_complete(data: Dict[str, Any])
    def on_receive_start(data: Dict[str, Any])
    def on_receive_complete(data: Dict[str, Any])
    def on_error(error: Exception, context: Dict[str, Any])
```

### MetricsCollector

```python
class MetricsCollector(Monitor):
    def __init__(
        enabled: bool = True,
        collect_interval_ms: int = 1000,
        log_level: str = "INFO"
    )
    
    def get_statistics() -> Statistics
    def reset()
    def export_json(output_path: str)
    def export_csv(output_path: str)
    def print_summary()
```

### DataLogger

```python
class DataLogger(Monitor):
    def __init__(
        output_file: str = "agent_data.log",
        format: str = "jsonl",  # "json", "jsonl", "csv"
        log_inputs: bool = True,
        log_outputs: bool = True,
        sample_rate: float = 1.0,
        include_data_samples: bool = False,
        max_sample_size: int = 10,
        enabled: bool = True,
        log_level: str = "INFO"
    )
    
    def close()
```

### DataInspector

```python
class DataInspector(Monitor):
    def __init__(
        validate_formats: bool = True,
        check_ranges: bool = True,
        detect_anomalies: bool = True,
        enabled: bool = True,
        log_level: str = "INFO"
    )
    
    def get_report() -> ValidationReport
    def reset()
```

### Profiler

```python
class Profiler(Monitor):
    def __init__(
        enabled: bool = True,
        track_operations: Optional[List[str]] = None,
        log_level: str = "INFO"
    )
    
    def get_profile() -> Profile
    def reset()
```

### Utility Functions

```python
def enable_monitoring(
    brain_input: Optional[Any] = None,
    brain_output: Optional[Any] = None,
    collect_metrics: bool = True,
    log_file: Optional[str] = None,
    log_format: str = "jsonl",
    log_level: str = "INFO",
    show_dashboard: bool = False
) -> tuple[MetricsCollector, DataLogger]

@contextmanager
def monitor_session(
    brain_input: Optional[Any] = None,
    brain_output: Optional[Any] = None,
    collect_metrics: bool = True,
    log_file: Optional[str] = None,
    log_format: str = "jsonl",
    export_on_exit: bool = True
) -> MonitorSession
```

---

## Performance Considerations

### Overhead

| Component | Overhead | Notes |
|-----------|----------|-------|
| MetricsCollector | <1% | Minimal CPU/memory usage |
| DataLogger (100% sample) | ~5% | I/O bound, async recommended |
| DataLogger (10% sample) | <1% | Reduced sampling |
| DataInspector | ~2% | Validation overhead |
| Profiler | ~10% | Use only for debugging |
| **Disabled monitors** | **0%** | Zero overhead when `enabled=False` |

### Optimization Tips

**1. Use Sampling for High-Frequency Data:**
```python
logger = DataLogger(sample_rate=0.1)  # Log 10% of packets
```

**2. Disable When Not Needed:**
```python
metrics.disable()  # Zero overhead
# ... fast processing ...
metrics.enable()
```

**3. Selective Monitoring:**
```python
# Only monitor inputs
brain_input.attach_monitor(metrics)
# brain_output not monitored
```

**4. Export Asynchronously:**
```python
import threading

def async_export():
    threading.Thread(target=metrics.export_json, args=("metrics.json",)).start()
```

**5. Limit Data Samples:**
```python
logger = DataLogger(
    include_data_samples=False  # Don't log actual data
)
```

---

## Troubleshooting

### Issue: Monitors Not Receiving Callbacks

**Symptom**: Monitors attached but no data collected.

**Solution**: Ensure monitors attached before calling `send()`/`receive()`:
```python
# ✅ Correct order
brain_input.attach_monitor(metrics)
brain_input.connect()
brain_input.send()

# ❌ Wrong order
brain_input.send()
brain_input.attach_monitor(metrics)  # Too late!
```

### Issue: Log Files Not Created

**Symptom**: No output files generated.

**Solution**: Call `logger.close()` to flush data:
```python
logger = DataLogger(output_file="data.jsonl", format="jsonl")
brain_input.attach_monitor(logger)

# ... run agent ...

logger.close()  # Required to flush data!
```

### Issue: High Memory Usage

**Symptom**: Memory grows over time.

**Solution**: Use JSONL format instead of JSON:
```python
# ❌ JSON accumulates in memory
logger = DataLogger(format="json")

# ✅ JSONL streams to disk
logger = DataLogger(format="jsonl")
```

### Issue: Performance Degradation

**Symptom**: Agent runs slower with monitoring enabled.

**Solution**: Reduce sampling or disable profiler:
```python
# Reduce logging overhead
logger = DataLogger(sample_rate=0.1)  # 10% sampling

# Disable profiler (highest overhead)
# profiler = Profiler()  # Comment out
```

### Issue: Validation Errors Don't Match Reality

**Symptom**: Inspector reports errors but data seems correct.

**Solution**: Adjust validation thresholds:
```python
# Inspector may be too strict
inspector = DataInspector(
    validate_formats=True,
    check_ranges=False,  # Disable range checking
    detect_anomalies=False  # Disable anomaly detection
)
```

---

## Best Practices

### 1. Start Simple

Begin with basic metrics collection:
```python
from feagi.pns.observability import enable_monitoring

metrics, logger = enable_monitoring(log_file="agent.log")
```

### 2. Add Validation During Development

Use DataInspector to catch issues early:
```python
inspector = DataInspector()
brain_input.attach_monitor(inspector)

# Check periodically
if inspector.get_report().has_errors():
    print("⚠️ Data validation errors detected!")
```

### 3. Profile Before Optimizing

Use Profiler to identify real bottlenecks:
```python
profiler = Profiler()
brain_input.attach_monitor(profiler)

# ... run agent ...

profile.print_summary()
```

### 4. Use Context Managers for Sessions

Automatic cleanup with context managers:
```python
with monitor_session(log_file="session.log") as session:
    # Monitoring automatically enabled
    run_agent()
    # Automatically exports and closes on exit
```

### 5. Export Metrics in CI/CD

Integrate with test pipelines:
```python
metrics.export_json("test_metrics.json")

# In CI/CD, check metrics against baseline
```

### 6. Log at Appropriate Levels

Use `log_level` to control verbosity:
```python
# Development: verbose logging
metrics = MetricsCollector(log_level="DEBUG")

# Production: minimal logging
metrics = MetricsCollector(log_level="WARNING")
```

### 7. Clean Up Resources

Always close loggers:
```python
try:
    # Agent code
    pass
finally:
    logger.close()  # Ensure data is flushed
```

### 8. Sample High-Frequency Data

Reduce overhead for high-frequency agents:
```python
# Sample 10% for 100Hz+ agents
logger = DataLogger(sample_rate=0.1)
```

### 9. Separate Concerns

Use different monitors for different purposes:
```python
# Development
dev_logger = DataLogger(output_file="dev.log", include_data_samples=True)

# Production
prod_metrics = MetricsCollector()
```

### 10. Document Monitoring Setup

Include monitoring in agent documentation:
```python
"""
Agent Monitoring:
- Metrics: Exported every 5 minutes to /var/log/metrics/
- Logs: JSONL format, 10% sampling
- Validation: Enabled for anomaly detection
"""
```

---

## Summary

The FEAGI PNS Observability module provides comprehensive tools for monitoring, debugging, and optimizing FEAGI agents:

- **MetricsCollector**: Track performance metrics
- **DataLogger**: Log structured data
- **DataInspector**: Validate and detect issues
- **Profiler**: Identify performance bottlenecks

Start with simple monitoring using `enable_monitoring()`, then add more sophisticated monitoring as needed. Always consider performance overhead and use sampling for high-frequency agents.

For questions or issues, refer to the API reference or contact the FEAGI development team.

---

**Next Steps:**
- Try the [Example Scripts](./observability-examples/)
- Review the [API Reference](#api-reference)
- Join the FEAGI community for support

