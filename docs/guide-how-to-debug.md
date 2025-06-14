# FEAGI Debugging Guide

*Last Updated: May 24, 2025*

This comprehensive guide covers all aspects of debugging FEAGI, from basic command line flags to advanced troubleshooting techniques.

## Overview

FEAGI provides multiple debugging approaches to help developers and researchers understand system behavior:

- **Command Line Debug Flags**: Real-time debugging during execution
- **Environment Variables**: Fine-grained debug control
- **Log Level Configuration**: Detailed system logging
- **Test Mode Integration**: Debugging during validation
- **Performance Monitoring**: System health and performance analysis

## Quick Start

### Basic Debugging Commands

```bash
# Debug neural processing (fire queue contents every burst)
python -m feagi.main --debug-npu

# Debug API requests and responses
python -m feagi.main --debug-api

# Combined debugging with detailed logging
python -m feagi.main --debug-api --debug-npu --log-level DEBUG

# Test mode with debugging
python -m feagi.main --test --debug-npu --test-duration 30
```

## Command Line Debug Flags

### NPU Debugging (`--debug-npu`)

Enables detailed neural processing unit debugging with fire queue visualization.

#### What It Shows:
- **Burst Information**: Current burst number and timing
- **Global Fire Summary**: Total neurons firing and target frequency
- **Per-Area Breakdown**: Neurons firing in each cortical area with percentages
- **Neuron IDs**: Individual firing neuron identifiers
- **Power Injection Stats**: Special area injection information
- **Recent Activity**: Historical firing rate trends

#### Sample Output:
```
🔥 ===== NPU DEBUG - BURST 1247 =====
📊 Global Fire Summary:
   Total firing neurons: 23
   Burst frequency: 100.0Hz target
🧠 Per-Area Breakdown (5 active areas):
   visual_cortex: 8 neurons (34.8%) - [1001, 1005, 1012, 1023, 1034, 1045, 1056, 1067]
   motor_pwr: 5 neurons (21.7%) - [2001, 2002, 2003, 2004, 2005]
   attention: 4 neurons (17.4%) - [3012, 3023, 3034, 3045]
⚡ Power Injection: 8 neurons from 2 power areas
📈 Recent Activity:
   Average firing rate: 18.5 neurons/burst
   Peak firing: 35 neurons
🔥 ========================================
```

#### Use Cases:
- **Neural Activity Analysis**: See which areas are most/least active
- **Power Area Validation**: Verify special area injection is working
- **Performance Monitoring**: Check if burst frequency targets are met
- **Genome Validation**: Ensure neural pathways work as expected

### API Debugging (`--debug-api`)

Enables comprehensive HTTP request/response logging with enhanced formatting and detailed tracking.

#### Enhanced Features:
- **🆔 Unique Request IDs**: Each request gets a unique 8-character ID for correlation
- **🔵 Detailed Request Logging**: Method, URL, headers, query params, path params, body
- **🟢 Comprehensive Response Logging**: Status, headers, body, timing information
- **🔴 Error Tracking**: Exception details with request correlation
- **🎨 Color-Coded Output**: Visual distinction between requests, responses, and errors
- **🔒 Security**: Automatic masking of sensitive headers (authorization, cookies, API keys)
- **📄 Pretty JSON**: Automatic JSON formatting for readable output
- **⏱️ Performance Timing**: Request duration tracking in milliseconds

#### What It Shows:
- **HTTP Requests**: Method, URL, client info, headers (with sensitive data masked)
- **Request Bodies**: Pretty-printed JSON or truncated text content
- **Query/Path Parameters**: All URL parameters clearly displayed
- **Response Data**: Status codes, headers, formatted response bodies
- **Timing Information**: Request processing duration
- **Error Context**: Detailed exception information with request correlation

#### Enhanced Output Format:
```
🔵 [API-DEBUG] ═══════════════════════════════════════
🔵 [API-DEBUG] REQUEST START [ID: A1B2C3D4]
🔵 [API-DEBUG] ═══════════════════════════════════════
🔵 [API-DEBUG] Method: POST
🔵 [API-DEBUG] URL: http://localhost:8001/v1/cortical_mapping/mapping_properties
🔵 [API-DEBUG] Path: /v1/cortical_mapping/mapping_properties
🔵 [API-DEBUG] Client: 127.0.0.1:54321
🔵 [API-DEBUG] Headers:
🔵 [API-DEBUG]   content-type: application/json
🔵 [API-DEBUG]   authorization: ***MASKED***
🔵 [API-DEBUG] Request Body (JSON):
🔵 [API-DEBUG]   {
🔵 [API-DEBUG]     "source_area": "power",
🔵 [API-DEBUG]     "destination_area": "central_vision"
🔵 [API-DEBUG]   }

🟢 [API-DEBUG] ═══════════════════════════════════════
🟢 [API-DEBUG] RESPONSE [ID: A1B2C3D4]
🟢 [API-DEBUG] ═══════════════════════════════════════
🟢 [API-DEBUG] Status: 200 ✅
🟢 [API-DEBUG] Duration: 45.23ms
🟢 [API-DEBUG] Response Headers:
🟢 [API-DEBUG]   content-type: application/json
🟢 [API-DEBUG] Response Body (JSON):
🟢 [API-DEBUG]   {
🟢 [API-DEBUG]     "success": true,
🟢 [API-DEBUG]     "connections_created": 27
🟢 [API-DEBUG]   }
🟢 [API-DEBUG] ═══════════════════════════════════════
🟢 [API-DEBUG] COMPLETED [ID: A1B2C3D4] POST /v1/cortical_mapping/mapping_properties → 200 (45.23ms)
🟢 [API-DEBUG] ═══════════════════════════════════════
```

#### Use Cases:
- **API Development**: Debug endpoint behavior and data flow with detailed visibility
- **Client Integration**: Troubleshoot client-server communication with request correlation
- **Performance Analysis**: Identify slow API operations with precise timing
- **Authentication Issues**: Debug login and permission problems (with secure header masking)
- **Data Flow Debugging**: Track request/response data transformations
- **Error Investigation**: Correlate errors with specific requests using unique IDs

### Combined Debugging

```bash
# Maximum debugging visibility
python -m feagi.main --debug-api --debug-npu --log-level DEBUG

# With custom configuration
python -m feagi.main --debug-api --debug-npu --config /path/to/debug_config.toml

# Remote debugging setup
python -m feagi.main --debug-api --debug-npu --host 0.0.0.0 --api-port 8001
```

## Environment Variables

### Debug Environment Variables

```bash
# Enable specific debugging features
export FEAGI_DEBUG_API=1        # Same as --debug-api
export FEAGI_DEBUG_NPU=1        # Same as --debug-npu

# System-level debugging
export FEAGI_LOG_LEVEL=DEBUG    # Detailed system logging
export FEAGI_DEBUG=1            # General debug mode

# Skip version checks for development
export FEAGI_SKIP_VERSION_CHECK=1
```

### Using in Tests

```bash
# NPU debugging during pytest runs
FEAGI_DEBUG_NPU=1 python -m pytest tests/npu/ -v -s

# API debugging during integration tests
FEAGI_DEBUG_API=1 python -m pytest tests/api/ -v -s

# Combined debugging in test environment
FEAGI_DEBUG_API=1 FEAGI_DEBUG_NPU=1 python -m pytest -v -s
```

## Log Level Configuration

### Log Levels

| Level | Description | Use Case |
|-------|-------------|----------|
| `DEBUG` | Detailed information for diagnosing problems | Development, debugging |
| `INFO` | General information about system operation | Normal operation, monitoring |
| `WARNING` | Warnings about potential issues | Production monitoring |
| `ERROR` | Error conditions that need attention | Production alerts |

### Configuration Methods

```bash
# Command line
python -m feagi.main --log-level DEBUG

# Environment variable
export FEAGI_LOG_LEVEL=DEBUG

# Programmatic configuration
import logging
logging.getLogger('feagi').setLevel(logging.DEBUG)
```

### Module-Specific Logging

```python
# Enable debug logging for specific modules
import logging

# NPU-specific debugging
logging.getLogger('feagi.npu.special_area_handler').setLevel(logging.DEBUG)
logging.getLogger('feagi.npu.fcl_injection_service').setLevel(logging.DEBUG)
logging.getLogger('feagi.npu.burst_engine').setLevel(logging.DEBUG)

# API-specific debugging
logging.getLogger('feagi.api').setLevel(logging.DEBUG)

# BDU-specific debugging
logging.getLogger('feagi.bdu').setLevel(logging.DEBUG)
```

## Debugging Common Scenarios

### 1. Neural Activity Not Working

**Symptoms**: No neurons firing, unexpected firing patterns

**Debug Approach**:
```bash
# 1. Check neural activity with NPU debugging
python -m feagi.main --debug-npu

# 2. Verify genome loading
python -m feagi.main --log-level DEBUG --debug-npu

# 3. Check specific areas
FEAGI_DEBUG_NPU=1 python -c "
from feagi.bdu.connectome_manager import ConnectomeManager
cm = ConnectomeManager.instance()
print('Cortical areas:', list(cm.cortical_areas.keys()))
"
```

**What to Look For**:
- Zero firing neurons across all bursts
- Missing cortical areas in breakdown
- Power area injection not working
- Frequency much lower than target

### 2. API Endpoints Not Responding

**Symptoms**: API timeouts, 500 errors, connection refused

**Debug Approach**:
```bash
# 1. Enable API debugging
python -m feagi.main --debug-api

# 2. Check port conflicts
lsof -i :8001
ss -tuln | grep 8001

# 3. Test basic connectivity
curl -v http://localhost:8001/v1/system/health_check
```

**What to Look For**:
- Port binding errors
- Request/response mismatches
- Authentication failures
- Middleware errors

### 3. Performance Issues

**Symptoms**: Slow burst rates, high latency, memory issues

**Debug Approach**:
```bash
# 1. Monitor burst performance
python -m feagi.main --debug-npu --log-level DEBUG

# 2. Check system resources
python -m feagi.main --debug-api --debug-npu | grep -E "(frequency|memory|timing)"

# 3. Profile specific operations
python -m cProfile -o feagi_profile.prof -m feagi.main --test --debug-npu
```

**What to Look For**:
- Actual vs target frequency in NPU debug output
- Large numbers of firing neurons (potential performance bottleneck)
- Memory allocation patterns
- API response times

## Tips and Best Practices

### General Debugging Tips

1. **Start Simple**: Begin with basic debug flags before adding complexity
2. **Save Output**: Redirect debug output to files for analysis
   ```bash
   python -m feagi.main --debug-npu > debug_output.log 2>&1
   ```
3. **Use Timestamps**: Include timestamps in logs for timing analysis
4. **Document Issues**: Keep a debugging log with symptoms and solutions

### Performance Tips

1. **Monitor Resources**: Watch CPU, memory, and disk usage during debugging
2. **Use Test Mode**: Prefer test mode for debugging to avoid side effects
3. **Limit Output**: Use grep/awk to filter debug output for specific issues
4. **Profile Regularly**: Run performance profiles to catch regressions early

## Common Issues and Solutions

### Issue: "No neurons firing"

**Symptoms**: NPU debug shows 0 neurons firing every burst

**Solutions**:
1. Check genome loading: `--log-level DEBUG`
2. Verify cortical area configuration
3. Check input stimulation
4. Validate synaptic connections

### Issue: "API requests timing out"

**Symptoms**: HTTP requests fail with timeout errors

**Solutions**:
1. Check port conflicts: `lsof -i :8001`
2. Verify API server startup: `--debug-api`
3. Test basic connectivity: `curl localhost:8001/v1/system/health_check`
4. Check firewall/network settings

### Issue: "Debug output not appearing"

**Symptoms**: Debug flags don't show expected output

**Solutions**:
1. Verify flag syntax: `--debug-npu` (with double dashes)
2. Check environment variables: `echo $FEAGI_DEBUG_NPU`
3. Use with stdout: `python -m feagi.main --debug-npu -s`
4. Check log levels and buffering

## Related Documentation

- [Usage Guide](guide-usage.md) - Basic FEAGI usage and configuration
- [NPU Documentation](../feagi/npu/README.md) - Neural Processing Unit details
- [API Testing Guide](../feagi/api/guide-api-testing.md) - API-specific testing approaches
- [Testing Documentation](../tests/README.md) - Test development and execution
- [Contribution Guide](guide-contribution.md) - Development workflow and standards
