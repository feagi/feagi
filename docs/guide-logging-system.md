# FEAGI Logging System Guide

*Last Updated: January 15, 2025*

## Purpose

This guide explains the FEAGI logging system architecture, CLI flag behavior, and best practices for debugging and monitoring. It covers the enhanced logging system that provides granular control over log levels for different system modules.

## Overview

The FEAGI logging system provides two levels of control:

1. **Global Log Level**: Sets the baseline logging level for all system components
2. **Module-Specific Debug Overrides**: Individual debug flags that override the global level for specific subsystems

This dual-level approach allows developers to focus on specific areas of interest while maintaining appropriate log levels for the rest of the system.

## CLI Logging Flags

### Global Log Level Control

```bash
--log-level {DEBUG,INFO,WARNING,ERROR}
```

Sets the baseline logging level for all FEAGI modules. This affects every logger in the system unless overridden by specific debug flags.

**Examples:**
```bash
# Set all modules to DEBUG level (very verbose)
python -m feagi.main --log-level DEBUG

# Set all modules to WARNING level (minimal output)
python -m feagi.main --log-level WARNING
```

### Module-Specific Debug Flags

Individual debug flags override the global log level for their respective modules, setting them to DEBUG level regardless of the global setting.

#### Neural Processing Unit (NPU) Debugging
```bash
--debug-npu
```

**Affected Modules:**
- `feagi.npu.*` - All NPU components
- `feagi.npu.burst_engine` - Core burst processing
- `feagi.npu.fcl_manager` - Fire candidate list management
- `feagi.npu.fcl_injection_service` - Special area injection
- `feagi.npu.special_area_handler` - Special area processing
- `feagi.npu.memory_processor` - Memory system processing
- `feagi.npu.fq_sampler` - Fire queue sampling

**Use Cases:**
- Debugging neural firing patterns
- Investigating burst engine performance
- Analyzing memory system behavior
- Troubleshooting special area processing

#### API Debugging
```bash
--debug-api
```

**Affected Modules:**
- `feagi.api.*` - All API components
- `feagi.api.rest` - REST API endpoints
- `feagi.api.core` - Core API services
- `feagi.api.gateway` - API gateway functionality
- `feagi.api.protocols` - Protocol implementations
- `feagi.api.transport` - Transport layer
- `feagi.api.zmq` - ZeroMQ API components

**Use Cases:**
- Debugging API request/response cycles
- Investigating protocol issues
- Analyzing transport layer problems
- Troubleshooting gateway functionality

#### Brain Development Unit (BDU) Debugging
```bash
--debug-bdu
```

**Affected Modules:**
- `feagi.bdu.*` - All BDU components
- `feagi.bdu.connectivity` - Cortical mapping and connections
- `feagi.bdu.embryogenesis` - Neural development processes
- `feagi.bdu.models` - Brain region models
- `feagi.bdu.utils` - BDU utilities

**Use Cases:**
- Debugging synaptogenesis processes
- Investigating cortical mapping issues
- Analyzing brain development phases
- Troubleshooting connectivity problems

#### ZeroMQ Communication Debugging
```bash
--debug-zmq-inbound   # Inbound ZMQ traffic
--debug-zmq-outbound  # Outbound ZMQ traffic
```

**Affected Modules:**
- `feagi.api.zmq.*` - ZeroMQ components
- `feagi.api.zmq.streams` - Data streaming
- `feagi.api.zmq.neural` - Neural data communication
- `feagi.api.zmq.memory` - Memory data communication
- `feagi.api.zmq.patterns` - Communication patterns

**Use Cases:**
- Debugging message flow issues
- Investigating communication bottlenecks
- Analyzing data serialization problems
- Troubleshooting network connectivity

#### Memory System Debugging
```bash
--debug-mem
```

**Affected Modules:**
- `feagi.npu.memory_processor` - NPU memory processing
- `feagi.bdu.models.memory` - Memory models
- `feagi.core.memory` - Core memory functionality

**Use Cases:**
- Debugging memory neuron creation
- Investigating pattern detection
- Analyzing long-term memory conversion
- Troubleshooting memory system performance

## Usage Patterns

### Common Debugging Scenarios

#### Scenario 1: Focus on Neural Processing
```bash
# Keep most logs quiet, but debug NPU in detail
python -m feagi.main --log-level WARNING --debug-npu
```

**Result:**
- NPU modules: DEBUG level (detailed output)
- All other modules: WARNING level (minimal output)

#### Scenario 2: API Development
```bash
# Normal logging with detailed API debugging
python -m feagi.main --log-level INFO --debug-api
```

**Result:**
- API modules: DEBUG level (detailed request/response logging)
- All other modules: INFO level (normal output)

#### Scenario 3: Brain Development Analysis
```bash
# Focus on brain development with minimal noise
python -m feagi.main --log-level ERROR --debug-bdu --debug-mem
```

**Result:**
- BDU modules: DEBUG level (detailed development logging)
- Memory modules: DEBUG level (detailed memory logging)
- All other modules: ERROR level (errors only)

#### Scenario 4: Communication Troubleshooting
```bash
# Debug both directions of ZMQ communication
python -m feagi.main --log-level WARNING --debug-zmq-inbound --debug-zmq-outbound
```

**Result:**
- ZMQ modules: DEBUG level (detailed communication logging)
- All other modules: WARNING level (minimal output)

#### Scenario 5: Full System Debug
```bash
# Everything at debug level
python -m feagi.main --log-level DEBUG
```

**Result:**
- All modules: DEBUG level (maximum verbosity)

### Performance Considerations

#### Recommended Practices

1. **Production Environments**: Use `--log-level WARNING` or `--log-level ERROR`
2. **Development**: Use `--log-level INFO` with specific debug flags as needed
3. **Debugging**: Use `--log-level WARNING` with targeted debug flags
4. **Full Debug**: Only use `--log-level DEBUG` when necessary (very verbose)

#### Log Level Impact

| Level | Performance Impact | Use Case |
|-------|-------------------|----------|
| ERROR | Minimal | Production monitoring |
| WARNING | Low | Production with issue tracking |
| INFO | Moderate | Development and testing |
| DEBUG | High | Detailed debugging only |

## Integration with Test Mode

Debug flags work seamlessly with FEAGI's test modes:

```bash
# Test mode with NPU debugging
python -m feagi.main --test --debug-npu --test-duration 30

# Test mode with API debugging
python -m feagi.main --test-mode-1 --debug-api --test-frequency 5
```

## Environment Variable Alternatives

For automated testing and CI/CD, some debug flags can be set via environment variables:

```bash
# Alternative to --debug-api
export FEAGI_DEBUG_API=1
python -m feagi.main

# Alternative to --debug-npu  
export FEAGI_DEBUG_NPU=1
python -m feagi.main
```

## Architecture Implementation

### Logger Hierarchy

The logging system uses Python's hierarchical logger structure:

```
feagi (root)
├── feagi.npu
│   ├── feagi.npu.burst_engine
│   ├── feagi.npu.fcl_manager
│   └── ...
├── feagi.api
│   ├── feagi.api.rest
│   ├── feagi.api.core
│   └── ...
└── feagi.bdu
    ├── feagi.bdu.connectivity
    ├── feagi.bdu.embryogenesis
    └── ...
```

### Override Mechanism

When a debug flag is enabled:

1. The global log level is applied to all loggers
2. Module-specific overrides are applied to the relevant hierarchies
3. Child loggers inherit the override unless explicitly set otherwise

### Implementation Location

The logging system implementation is located in:
- **Main Logic**: `feagi/main.py` - CLI argument processing and logger configuration
- **Logger Setup**: `feagi/utils/logger.py` - Logger creation and configuration
- **Configuration**: `feagi/config/toml_loader.py` - Configuration management

## Best Practices

### Development Workflow

1. **Start Broad**: Begin with `--log-level INFO` to understand general system behavior
2. **Focus Narrow**: Add specific debug flags for areas of interest
3. **Minimize Noise**: Use higher log levels (WARNING/ERROR) with targeted debug flags
4. **Document Findings**: Note useful flag combinations for future debugging

### Debugging Strategy

1. **Identify Symptoms**: Determine which subsystem is likely involved
2. **Enable Relevant Debug**: Use the appropriate debug flag for that subsystem
3. **Analyze Output**: Look for patterns, errors, or unexpected behavior
4. **Iterate**: Adjust log levels and flags based on findings

### Performance Optimization

1. **Avoid Full Debug in Production**: `--log-level DEBUG` generates massive logs
2. **Use Targeted Debugging**: Specific flags are more efficient than global debug
3. **Monitor Log Volume**: High-frequency operations can generate significant output
4. **Consider Log Rotation**: Implement log rotation for long-running processes

## Related Documentation

- [FEAGI Debugging Guide](guide-how-to-debug.md) - Comprehensive debugging techniques
- [System Architecture](arch-system-overview.md) - Overall system design
- [API Architecture](arch-api-decorator-architecture.md) - API system design
- [NPU Architecture](arch-npu.md) - Neural processing unit design
- [BDU Architecture](arch-bdu.md) - Brain development unit design

## Troubleshooting

### Common Issues

#### Debug Flags Not Working
- **Symptom**: Debug flag enabled but no debug output
- **Solution**: Check that the module is actually being used and creating log messages
- **Verification**: Enable `--log-level DEBUG` temporarily to see all output

#### Too Much Log Output
- **Symptom**: Overwhelming amount of log messages
- **Solution**: Increase global log level and use specific debug flags
- **Example**: Change from `--log-level DEBUG` to `--log-level WARNING --debug-npu`

#### Missing Expected Logs
- **Symptom**: Expected debug output not appearing
- **Solution**: Verify the correct debug flag for the module in question
- **Reference**: Check module mappings in this guide

### Support

For additional support with the logging system:
1. Check the [FEAGI Debugging Guide](guide-how-to-debug.md)
2. Review module-specific documentation in respective directories
3. Consult the system architecture documentation for component relationships
