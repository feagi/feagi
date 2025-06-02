# FEAGI ZMQ Debugging Guide

A comprehensive guide to debugging ZMQ traffic in FEAGI 2.0 with the high-performance debug system.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Debug Levels](#debug-levels)
3. [Filtering Options](#filtering-options)
4. [Performance Monitoring](#performance-monitoring)
5. [Common Debugging Scenarios](#common-debugging-scenarios)
6. [API Reference](#api-reference)
7. [Performance Considerations](#performance-considerations)
8. [Troubleshooting](#troubleshooting)

## Quick Start

### Enable Debug Output in Console

The fastest way to see ZMQ traffic is to enable console output:

```bash
# Option 1: Runtime API call (recommended)
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -H "Content-Type: application/json" \
  -d '{"outbound_enabled": true, "console_output": true, "debug_level": "summary"}'

# Option 2: Environment variables (requires restart)
export FEAGI_DEBUG_ZMQ_OUTBOUND=1
export FEAGI_DEBUG_ZMQ_CONSOLE=1
python3 feagi/main.py

# Option 3: Command line flags + runtime console enable
python3 feagi/main.py --debug-zmq-outbound
# Then enable console:
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"console_output": true}'
```

### Quick Status Check

```bash
curl -s http://localhost:8000/v1/debug/zmq/status | jq
```

## Debug Levels

### Level Descriptions

| Level | What You See | When to Use |
|-------|--------------|-------------|
| `off` | Nothing (zero overhead) | Production systems |
| `minimal` | Basic connection info | Health monitoring |
| `headers` | + Message metadata | Connection debugging |
| `summary` | + Data previews (200 chars) | **Most debugging tasks** |
| `full` | Complete data dumps | Deep protocol analysis |

### Setting Debug Levels

```bash
# Runtime API (immediate effect)
curl -X POST http://localhost:8000/v1/debug/zmq/level/summary

# Via configuration
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"debug_level": "headers"}'

# Environment variable (startup only)
export FEAGI_DEBUG_ZMQ_LEVEL=summary
```

### Example Output by Level

**Minimal Level:**
```
📤 ZMQ OUTBOUND [13:45:23.123] tcp://0.0.0.0:5562 (1024b)
```

**Headers Level:**
```
📤 ZMQ OUTBOUND [13:45:23.123]
   [TARGET] tcp://0.0.0.0:5562
   [TYPE] visualization
   [STATS] Frames: 2, Size: 1024b
   [TAG] Topic: 'brain_activity'
────────────────────────────────────────
```

**Summary Level (Recommended):**
```
📤 ZMQ OUTBOUND [13:45:23.123]
   [TARGET] tcp://0.0.0.0:5562
   [TYPE] visualization
   [STATS] Frames: 2, Size: 1024b
   [TAG] Topic: 'brain_activity'
   📄 Frame 0: TEXT: brain_activity
   📄 Frame 1: JSON: {"cortical_areas": {"v1": {"neurons": [1,0,1,0]}, ...}}
────────────────────────────────────────
```

## Filtering Options

### Message Type Filtering

Filter by the type of ZMQ traffic you want to debug:

```bash
# Only visualization traffic
curl -X POST http://localhost:8000/v1/debug/zmq/filter/messages \
  -d '["visualization"]'

# Multiple message types
curl -X POST http://localhost:8000/v1/debug/zmq/filter/messages \
  -d '["sensory", "motor", "visualization"]'

# Clear all filters (show everything)
curl -X POST http://localhost:8000/v1/debug/zmq/filter/messages \
  -d '[]'
```

**Available Message Types:**

| Stream | Purpose | Default Port |
|--------|---------|--------------|
| `sensory` | Incoming sensory data | Port 5558 |
| `motor` | Outgoing motor commands | Port 5564 |
| `visualization` | Neural activity broadcasting | Port 5562 |
| `rest` | REST API over ZMQ (primary interface) | Port 5563 |
| `heartbeat` | Client keepalive messages | Various |

### Endpoint Filtering

Filter by specific ZMQ endpoints (useful for isolating specific connections):

```bash
# Debug only visualization stream
curl -X POST http://localhost:8000/v1/debug/zmq/filter/endpoints \
  -d '["tcp://0.0.0.0:5562"]'

# Multiple endpoints
curl -X POST http://localhost:8000/v1/debug/zmq/filter/endpoints \
  -d '["tcp://0.0.0.0:5562", "tcp://localhost:5558"]'

# Clear endpoint filters
curl -X POST http://localhost:8000/v1/debug/zmq/filter/endpoints \
  -d '[]'
```

### Rate Limiting

Control how many debug messages are shown per second:

```bash
# Low noise (good for production monitoring)
curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/1

# Moderate (good for development)
curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/10

# High verbosity (debugging specific issues)
curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/100
```

## Performance Monitoring

### Debug System Statistics

Monitor the overhead and effectiveness of debugging:

```bash
curl -s http://localhost:8000/v1/debug/zmq/status | jq '.stats'
```

**Key Metrics:**
- `messages_logged`: Total debug messages captured
- `messages_filtered`: Messages filtered out by filters
- `debug_overhead_ms`: CPU time spent on debugging
- `rate_limited_messages`: Messages dropped due to rate limiting
- `uptime_seconds`: Debug system uptime

### Per-Endpoint Statistics

See which endpoints are generating the most traffic:

```bash
curl -s http://localhost:8000/v1/debug/zmq/endpoints | jq

# Example output:
{
  "endpoints": {
    "tcp://0.0.0.0:5562": {
      "messages_logged": 4013,
      "total_bytes": 235292,
      "rate_limited": 0
    },
    "tcp://0.0.0.0:5563": {
      "messages_logged": 75,
      "total_bytes": 27979,
      "rate_limited": 5
    }
  }
}
```

### Reset Statistics

Clear all counters for fresh measurement:

```bash
curl -X POST http://localhost:8000/v1/debug/zmq/reset-stats
```

## Common Debugging Scenarios

### 1. "Why isn't my agent receiving data?"

**Check outbound motor stream:**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{
    "outbound_enabled": true,
    "debug_level": "summary",
    "message_filters": ["motor"],
    "console_output": true,
    "rate_limit_per_second": 10
  }'
```

Look for:
- Messages being sent to the correct endpoint
- Data content looks reasonable
- Timestamps are recent

### 2. "Why isn't FEAGI receiving my sensor data?"

**Check inbound sensory stream:**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{
    "inbound_enabled": true,
    "debug_level": "headers",
    "message_filters": ["sensory"],
    "console_output": true
  }'
```

Look for:
- Messages arriving at the sensory endpoint (5558)
- Correct message format
- No decoding errors

### 3. "Visualization is slow/choppy"

**Monitor visualization stream performance:**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{
    "outbound_enabled": true,
    "debug_level": "minimal",
    "message_filters": ["visualization"],
    "console_output": true,
    "rate_limit_per_second": 5
  }'
```

Look for:
- Regular message timing
- Message sizes (large messages = slower updates)
- Gaps in message flow

### 4. "High CPU usage investigation"

**Monitor all traffic with rate limiting:**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{
    "inbound_enabled": true,
    "outbound_enabled": true,
    "debug_level": "minimal",
    "console_output": true,
    "rate_limit_per_second": 2
  }'

# Check which endpoints have highest traffic
curl -s http://localhost:8000/v1/debug/zmq/endpoints | jq
```

### 5. "Deep protocol analysis"

**Examine actual data content (use sparingly):**
```bash
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{
    "outbound_enabled": true,
    "debug_level": "full",
    "endpoint_filters": ["tcp://0.0.0.0:5562"],
    "rate_limit_per_second": 1,
    "console_output": true
  }'
```

**⚠️ Warning:** Full level can generate massive output!

## API Reference

### Configuration Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/v1/debug/zmq/status` | GET | Current debug status |
| `/v1/debug/zmq/configure` | POST | Complete configuration |
| `/v1/debug/zmq/enable` | POST | Quick enable |
| `/v1/debug/zmq/disable` | POST | Quick disable |
| `/v1/debug/zmq/level/{level}` | POST | Set debug level |
| `/v1/debug/zmq/filter/messages` | POST | Set message filters |
| `/v1/debug/zmq/filter/endpoints` | POST | Set endpoint filters |
| `/v1/debug/zmq/rate-limit/{limit}` | POST | Set rate limit |
| `/v1/debug/zmq/endpoints` | GET | Per-endpoint stats |
| `/v1/debug/zmq/reset-stats` | POST | Reset statistics |
| `/v1/debug/zmq/help` | GET | API documentation |

### Configuration Object

```json
{
  "inbound_enabled": true,           // Enable inbound debugging
  "outbound_enabled": true,          // Enable outbound debugging  
  "debug_level": "summary",          // off|minimal|headers|summary|full
  "message_filters": ["visualization"], // Empty = all types
  "endpoint_filters": [],            // Empty = all endpoints
  "rate_limit_per_second": 10,       // Messages per second limit
  "console_output": true             // true = console, false = log files
}
```

### Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `FEAGI_DEBUG_ZMQ_INBOUND` | Enable inbound debugging | `1` |
| `FEAGI_DEBUG_ZMQ_OUTBOUND` | Enable outbound debugging | `1` |
| `FEAGI_DEBUG_ZMQ_CONSOLE` | Enable console output | `1` |
| `FEAGI_DEBUG_ZMQ_LEVEL` | Debug verbosity level | `summary` |
| `FEAGI_DEBUG_ZMQ_FILTER_TYPES` | Message type filters | `visualization,motor` |

## Performance Considerations

### Debug System Overhead

| Configuration | Overhead | Use Case |
|---------------|----------|----------|
| Disabled | **0%** (zero overhead) | Production |
| Minimal + File | <0.01% | Production monitoring |
| Summary + File | <0.1% | Development |
| Summary + Console | <0.2% | Active debugging |
| Full + Console | 1-5% | Deep analysis only |

### Best Practices

**For Production:**
- Keep debugging disabled unless investigating issues
- Use `minimal` level with file output if monitoring needed
- Enable rate limiting (1-5 msg/sec)

**For Development:**
- Use `summary` level with console output
- Apply message type filters to reduce noise
- Use moderate rate limiting (10-20 msg/sec)

**For Deep Analysis:**
- Use `full` level only for specific endpoints
- Heavy rate limiting (1 msg/sec)
- Short time periods only

### Memory Usage

- Debug system uses bounded queues (max 1000 entries)
- Automatic cleanup prevents memory leaks
- Statistics tracking has minimal overhead
- Console logger uses separate thread

## Troubleshooting

### "I enabled debugging but see no output"

1. **Check if debugging is actually enabled:**
   ```bash
   curl -s http://localhost:8000/v1/debug/zmq/status | jq '.inbound_enabled, .outbound_enabled'
   ```

2. **Check console output setting:**
   ```bash
   curl -s http://localhost:8000/v1/debug/zmq/status | jq '.console_output'
   ```

3. **Enable console output if needed:**
   ```bash
   curl -X POST http://localhost:8000/v1/debug/zmq/configure \
     -d '{"console_output": true}'
   ```

### "Too much debug output"

1. **Enable rate limiting:**
   ```bash
   curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/5
   ```

2. **Use message filtering:**
   ```bash
   curl -X POST http://localhost:8000/v1/debug/zmq/filter/messages \
     -d '["visualization"]'
   ```

3. **Lower debug level:**
   ```bash
   curl -X POST http://localhost:8000/v1/debug/zmq/level/minimal
   ```

### "Debug messages going to log files instead of console"

```bash
# Enable console output
curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -d '{"console_output": true}'

# Verify setting
curl -s http://localhost:8000/v1/debug/zmq/status | jq '.console_output'
```

### "High debug overhead"

1. **Check current overhead:**
   ```bash
   curl -s http://localhost:8000/v1/debug/zmq/status | jq '.stats.debug_overhead_ms'
   ```

2. **Reduce verbosity:**
   ```bash
   curl -X POST http://localhost:8000/v1/debug/zmq/level/minimal
   ```

3. **Increase rate limiting:**
   ```bash
   curl -X POST http://localhost:8000/v1/debug/zmq/rate-limit/1
   ```

4. **Disable when not needed:**
   ```bash
   curl -X POST http://localhost:8000/v1/debug/zmq/disable
   ```

### "API calls failing"

1. **Check if FEAGI is running:**
   ```bash
   curl -s http://localhost:8000/health
   ```

2. **Check if debug API is available:**
   ```bash
   curl -s http://localhost:8000/v1/debug/info
   ```

3. **Check JSON formatting:**
   ```bash
   # Use proper JSON formatting
   curl -X POST http://localhost:8000/v1/debug/zmq/configure \
     -H "Content-Type: application/json" \
     -d '{"console_output": true}'
   ```

## Examples

### Complete Setup for Visualization Debugging

```bash
#!/bin/bash
# Complete setup script for debugging visualization issues

echo "Configuring ZMQ debug system for visualization debugging..."

curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -H "Content-Type: application/json" \
  -d '{
    "outbound_enabled": true,
    "inbound_enabled": false,
    "debug_level": "summary", 
    "message_filters": ["visualization"],
    "console_output": true,
    "rate_limit_per_second": 10
  }'

echo "Debug system configured. Watch console for ZMQ traffic..."
echo "To stop debugging: curl -X POST http://localhost:8000/v1/debug/zmq/disable"
```

### Monitor All Traffic Script

```bash
#!/bin/bash
# Monitor all ZMQ traffic with rate limiting

curl -X POST http://localhost:8000/v1/debug/zmq/configure \
  -H "Content-Type: application/json" \
  -d '{
    "inbound_enabled": true,
    "outbound_enabled": true,
    "debug_level": "headers",
    "console_output": true,
    "rate_limit_per_second": 5
  }'

echo "Monitoring all ZMQ traffic. Press Ctrl+C to stop."
echo "Statistics: curl -s http://localhost:8000/v1/debug/zmq/endpoints | jq"
```

## Related Documentation

- [ZMQ Architecture](arch-zmq.md) - Complete ZMQ system architecture
- [REST API Documentation](../rest/README.md) - Debug API reference
- [Performance Tuning](../docs/performance.md) - System optimization
- [Troubleshooting Guide](../docs/troubleshooting.md) - General FEAGI debugging 