# FEAGI PNS Observability - Examples

This directory contains example scripts demonstrating the FEAGI PNS Observability module.

## Examples Overview

| Example | Description | Difficulty | Features |
|---------|-------------|------------|----------|
| `01_basic_metrics.py` | Basic metrics collection | ⭐ Easy | MetricsCollector, Export |
| `02_data_logging.py` | Structured data logging | ⭐ Easy | DataLogger, JSONL, CSV |
| `03_data_validation.py` | Data validation and anomaly detection | ⭐⭐ Medium | DataInspector, Validation |
| `04_performance_profiling.py` | Performance profiling and bottleneck detection | ⭐⭐ Medium | Profiler, Optimization |
| `05_comprehensive_monitoring.py` | All features combined | ⭐⭐⭐ Advanced | All monitors |
| `06_quick_start.py` | Simplest setup (one-liner) | ⭐ Easy | enable_monitoring() |

## Running the Examples

### Prerequisites

1. Install FEAGI Python SDK:
```bash
pip install feagi
```

2. Install Rust-backed SDK (for best performance):
```bash
pip install feagi_rust_py_libs
```

3. (Optional) Start FEAGI server:
```bash
# If testing with actual FEAGI instance
feagi start
```

**Note**: Examples work without a running FEAGI server (they won't send actual data but will demonstrate the monitoring features).

### Run an Example

```bash
cd examples/observability
python 01_basic_metrics.py
```

## Example Details

### Example 1: Basic Metrics Collection

**File**: `01_basic_metrics.py`

Demonstrates:
- Creating a MetricsCollector
- Attaching to brain_input and brain_output
- Running an agent
- Viewing collected metrics
- Exporting metrics to JSON and CSV

**Output Files**:
- `metrics_basic.json`
- `metrics_basic.csv`

**Usage**:
```bash
python 01_basic_metrics.py
```

### Example 2: Data Logging

**File**: `02_data_logging.py`

Demonstrates:
- Creating DataLogger instances
- Logging in JSONL and CSV formats
- Viewing log entries
- Analyzing logged data

**Output Files**:
- `data_log.jsonl`
- `data_log.csv`

**Usage**:
```bash
python 02_data_logging.py

# View logs
cat data_log.jsonl | jq '.'
```

### Example 3: Data Validation

**File**: `03_data_validation.py`

Demonstrates:
- Creating a DataInspector
- Testing with various data conditions
- Viewing validation reports
- Detecting anomalies

**Usage**:
```bash
python 03_data_validation.py
```

### Example 4: Performance Profiling

**File**: `04_performance_profiling.py`

Demonstrates:
- Creating a Profiler
- Running performance benchmarks
- Identifying bottlenecks
- Getting optimization suggestions

**Usage**:
```bash
python 04_performance_profiling.py
```

### Example 5: Comprehensive Monitoring

**File**: `05_comprehensive_monitoring.py`

Demonstrates:
- Using all monitors together
- Comprehensive observability
- Export in multiple formats
- Complete monitoring workflow

**Output Files**:
- `comprehensive_metrics.json`
- `comprehensive_metrics.csv`
- `comprehensive_data.jsonl`

**Usage**:
```bash
python 05_comprehensive_monitoring.py
```

### Example 6: Quick Start

**File**: `06_quick_start.py`

Demonstrates:
- One-liner monitoring setup
- Minimal configuration
- Quick results

**Output Files**:
- `quick_start_data.log`

**Usage**:
```bash
python 06_quick_start.py
```

## Common Patterns

### Pattern 1: Debug a Specific Issue

Use DataInspector and DataLogger:

```python
from feagi.pns.observability import DataInspector, DataLogger

inspector = DataInspector()
logger = DataLogger(output_file="debug.jsonl", include_data_samples=True)

brain_input.attach_monitor(inspector)
brain_input.attach_monitor(logger)

# Run problematic code...

report = inspector.get_report()
if report.has_errors():
    report.print_summary()
```

### Pattern 2: Benchmark Performance

Use MetricsCollector and Profiler:

```python
from feagi.pns.observability import MetricsCollector, Profiler

metrics = MetricsCollector()
profiler = Profiler()

brain_input.attach_monitor(metrics)
brain_input.attach_monitor(profiler)

# Run benchmark...

metrics.print_summary()
profiler.get_profile().print_summary()
```

### Pattern 3: Production Monitoring

Use lightweight metrics with periodic export:

```python
from feagi.pns.observability import MetricsCollector
import threading
import time

metrics = MetricsCollector()
brain_input.attach_monitor(metrics)

def export_periodically():
    while True:
        time.sleep(300)  # Every 5 minutes
        metrics.export_json(f"metrics_{int(time.time())}.json")

threading.Thread(target=export_periodically, daemon=True).start()
```

### Pattern 4: Quick Development Check

Use enable_monitoring() one-liner:

```python
from feagi.pns.observability import enable_monitoring

metrics, logger = enable_monitoring(log_file="dev.log")

# Your code...

metrics.print_summary()
logger.close()
```

## Troubleshooting

### Issue: "Module not found: feagi_rust_py_libs"

**Solution**: Install the Rust-backed SDK:
```bash
pip install feagi_rust_py_libs
```

### Issue: No output files created

**Solution**: Call `logger.close()` to flush data:
```python
logger.close()  # Required!
```

### Issue: High memory usage

**Solution**: Use JSONL format instead of JSON:
```python
logger = DataLogger(format="jsonl")  # Streams to disk
```

### Issue: Examples run but no FEAGI interaction

**Explanation**: Examples work without a running FEAGI server (demonstrating monitoring features). To test with actual FEAGI:
```bash
feagi start  # Start FEAGI server first
```

## Next Steps

1. Read the [Observability Usage Guide](../../docs/observability-usage-guide.md)
2. Review the [API Reference](../../docs/observability-usage-guide.md#api-reference)
3. Integrate monitoring into your own agents
4. Contribute examples for specific use cases

## Contributing

Have a useful monitoring pattern? Submit a pull request with:
1. New example script
2. Documentation in this README
3. Sample output (if applicable)

## Questions?

- Review the comprehensive guide: `docs/observability-usage-guide.md`
- Check the main SDK README: `README.md`
- Visit: https://feagi.org

