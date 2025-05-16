# FEAGI Usage Guide

This document provides detailed instructions on how to use FEAGI, including command-line options, configuration, and examples.

## Running FEAGI

FEAGI has a modular architecture with two main components:

1. **API Server**: Handles external HTTP requests and provides a REST API
2. **ZMQ Server**: Manages internal communication between components using ZeroMQ

## Command-Line Usage

### Unified Command

The recommended way to run FEAGI is with the unified command:

```bash
feagi
```

This starts:
- API server on 127.0.0.1:8000
- ZMQ publisher on 127.0.0.1:5556
- ZMQ subscriber on 127.0.0.1:5557

### Available Options

```
usage: feagi [-h] [--api-host API_HOST] [--api-port API_PORT] [--api-reload] [--zmq-host ZMQ_HOST] 
             [--zmq-pub-port ZMQ_PUB_PORT] [--zmq-sub-port ZMQ_SUB_PORT] 
             [--zmq-topics ZMQ_TOPICS [ZMQ_TOPICS ...]] [--zmq-auth] [--zmq-encryption] 
             [--config CONFIG] [--api-only] [--zmq-only]
```

#### General Options

| Option | Description |
|--------|-------------|
| `-h, --help` | Show help message |
| `--config CONFIG` | Path to configuration file |
| `--api-only` | Start only the API server |
| `--zmq-only` | Start only the ZMQ server |

#### API Server Options

| Option | Description | Default |
|--------|-------------|---------|
| `--api-host API_HOST` | API server host | 127.0.0.1 |
| `--api-port API_PORT` | API server port | 8000 |
| `--api-reload` | Enable auto-reload for development | False |

#### ZMQ Server Options

| Option | Description | Default |
|--------|-------------|---------|
| `--zmq-host ZMQ_HOST` | ZMQ server host | 127.0.0.1 |
| `--zmq-pub-port ZMQ_PUB_PORT` | ZMQ publisher port | 5556 |
| `--zmq-sub-port ZMQ_SUB_PORT` | ZMQ subscriber port | 5557 |
| `--zmq-topics TOPICS [TOPICS ...]` | ZMQ topics to support | neural, metrics, heartbeat |
| `--zmq-auth` | Enable ZMQ authentication | False |
| `--zmq-encryption` | Enable ZMQ encryption | False |

### Examples

```bash
# Run with default settings
feagi

# Run on a different port
feagi --api-port 8080

# Run with development auto-reload
feagi --api-reload

# Bind to all interfaces for remote access
feagi --api-host 0.0.0.0 --zmq-host 0.0.0.0

# Configure ZMQ with custom topics
feagi --zmq-topics neural control feedback visualization

# Start only the API server
feagi --api-only

# Start only the ZMQ server
feagi --zmq-only
```

## Component-Specific Commands

You can run the API and ZMQ servers independently:

### API Server

```bash
feagi-api
```

#### API Server Options

```
usage: feagi-api [-h] [--host HOST] [--port PORT] [--reload]
```

| Option | Description | Default |
|--------|-------------|---------|
| `-h, --help` | Show help message | |
| `--host HOST` | Host to run the server on | 127.0.0.1 |
| `--port PORT` | Port to run the server on | 8000 |
| `--reload` | Enable auto-reload | False |

### ZMQ Server

```bash
feagi-zmq
```

#### ZMQ Server Options

```
usage: feagi-zmq [-h] [--host HOST] [--pub-port PUB_PORT] [--sub-port SUB_PORT]
                 [--topics TOPICS [TOPICS ...]] [--auth] [--encryption]
                 [--config CONFIG]
```

| Option | Description | Default |
|--------|-------------|---------|
| `-h, --help` | Show help message | |
| `--host HOST` | Host to run the server on | 127.0.0.1 |
| `--pub-port PUB_PORT` | Publisher port | 5556 |
| `--sub-port SUB_PORT` | Subscriber port | 5557 |
| `--topics TOPICS [TOPICS ...]` | Topics to support | |
| `--auth` | Enable authentication | False |
| `--encryption` | Enable encryption | False |
| `--config CONFIG` | Path to configuration file | |

## Python Module Usage

You can run FEAGI directly as Python modules:

```bash
# Run the full system
python -m feagi.main

# Run only the API server
python -m feagi.api.server

# Run only the ZMQ server
python -m feagi.api.zmq.server
```

## Programmatic Usage

You can use FEAGI programmatically in your Python code:

```python
from feagi import create_feagi
from feagi.api.zmq import create_zmq_server
from feagi.core.resource_mgr import ResourceManager

# Create a complete FEAGI instance
feagi = create_feagi()
resource_mgr = feagi["resource_mgr"]

# Or create individual components
zmq_server = create_zmq_server(host="127.0.0.1", pub_port=5556, sub_port=5557)
zmq_server.start()

# Use the resource manager directly
resource_mgr = ResourceManager()
```

## API Documentation

Once the API server is running, you can access the interactive API documentation:
- Swagger UI: http://127.0.0.1:8000/docs
- ReDoc: http://127.0.0.1:8000/redoc

## Development Tools

### Memory Profiling

FEAGI includes built-in memory profiling tools:

```python
from feagi.utils.memory_profiler import MemoryProfiler

# Create a memory profiler
profiler = MemoryProfiler()

# Take a snapshot before operation
profiler.take_snapshot("before_operation")

# Run your code
# ...

# Take another snapshot
profiler.take_snapshot("after_operation")

# Compare snapshots to find memory growth
diff = profiler.compare_snapshots("before_operation", "after_operation")
print(diff)
```

### Benchmarking

FEAGI includes benchmarking tools for performance measurement:

```python
from feagi.utils.benchmarking import benchmark

# Use as a decorator
@benchmark
def my_function():
    # ...

# Or use the function directly
result = benchmark(my_function)
print(f"Execution time: {result.execution_time_ms} ms")
```

## Troubleshooting

### Common Issues

1. **Port already in use**: If ports are in use, try using different ports:
   ```bash
   feagi --api-port 8080 --zmq-pub-port 5566 --zmq-sub-port 5567
   ```

2. **Permission denied**: Make sure you have appropriate permissions to bind to the specified host/port.

3. **Dependencies missing**: Ensure all dependencies are installed:
   ```bash
   pip install -r requirements.txt
   ```

4. **ZMQ communication issues**: If ZMQ components aren't communicating, check firewall settings and ensure ports are open.

## Configuration

FEAGI can be configured using a YAML configuration file:

```bash
feagi --config config.yaml
```

Example configuration file:

```yaml
api:
  host: 127.0.0.1
  port: 8000
  reload: false

zmq:
  host: 127.0.0.1
  pub_port: 5556
  sub_port: 5557
  topics:
    - neural
    - metrics
    - heartbeat
  auth: false
  encryption: false

logging:
  level: INFO
  format: "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
  file: feagi.log
```

## Environment Variables

FEAGI also supports configuration via environment variables:

```bash
# Set API host and port
export FEAGI_API_HOST=0.0.0.0
export FEAGI_API_PORT=8080

# Set ZMQ configuration
export FEAGI_ZMQ_HOST=0.0.0.0
export FEAGI_ZMQ_PUB_PORT=5566
export FEAGI_ZMQ_SUB_PORT=5567

# Run FEAGI (will use environment variables)
feagi
``` 