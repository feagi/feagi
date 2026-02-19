# PNS Observability Examples

Examples for the PNS observability module: metrics, logging, profiling, and validation.

| Script | Purpose |
|--------|--------|
| `01_basic_metrics.py` | MetricsCollector, export to JSON/CSV |
| `02_data_logging.py` | DataLogger, JSONL and CSV logging |
| `03_data_validation.py` | DataInspector, validation and anomaly detection |
| `04_performance_profiling.py` | Profiler, bottlenecks and optimization |
| `05_comprehensive_monitoring.py` | Combined use of observability components |
| `06_quick_start.py` | Minimal setup with `enable_monitoring()` |

## Requirements

- Python 3.10+
- See `requirements.txt` in this folder. Base SDK; some scripts use Camera/Servo (same as simple_robot).

## Configuration

Connection uses environment variables (e.g. `FEAGI_AGENT_DESCRIPTOR_B64`, `FEAGI_HOST`, ports). See root examples README or individual script docstrings.

## Run

From this folder:

```bash
python 01_basic_metrics.py
python 06_quick_start.py
```

From the parent `examples/` directory:

```bash
python observability/01_basic_metrics.py
```

FEAGI must be running for examples that connect to the brain.
