# Dynamic Motor Area Registration

## Overview

This document describes the architectural improvement to support dynamic motor area registration in FEAGI Connector 2.

## Problem Statement

Previously, the registration system had a hardcoded list of motor cortical areas:

```python
"source_cortical_areas": ["omot00"]  # ❌ HARDCODED
```

This meant:
- Agents could only register with `omot00` regardless of what motor devices they actually registered
- Adding new motor areas (like `ogaz00` for gaze) was impossible
- The registration payload didn't reflect actual agent capabilities

## Solution Architecture

### 1. Registration Tracking

**Added callback mechanism to motor device types** (`motor_device_types.py`):
- `Percentage1D`, `Percentage4D`, and `MiscData` now support registration callbacks
- When `.register()` is called, it notifies the parent `MotorsProxy`

### 2. Cortical Area Mapping

**Added device-to-cortical-area mapping** (`motors_proxy.py`):
```python
DEVICE_TO_CORTICAL_PREFIX = {
    "gaze_absolute_linear": "ogaz",
    "miscellaneous_absolute": "omot",
    "positional_servo_absolute_linear": "oser",
    ...
}
```

When a device registers with cortical group N, it generates the full cortical area name (e.g., `"ogaz" + 00 = "ogaz00"`).

### 3. Dynamic Registration Payload

**Modified connection flow** (`feagi_interface.py` → `transport_interface/zmq.py`):
1. `FeagiAgent.brain_output.get_registered_cortical_areas()` returns list of registered areas
2. This list is passed to `connect_via_zmq(motor_cortical_areas=[...])`
3. Registration payload is built dynamically from actual registrations

## Usage Example

```python
from feagi_connector_2 import FeagiAgent
import feagi_rust_py_libs as frpl

feagi_agent = FeagiAgent()

# Register multiple motor types
feagi_agent.brain_output.miscellaneous_absolute.register(
    cortical_group=0, 
    number_of_channels=10,
    misc_dimensions=frpl.connector_core.data_types.descriptors.MiscDataDimensions(10, 1, 1)
)  # Registers: omot00

feagi_agent.brain_output.gaze_absolute_linear.register(
    cortical_group=0,
    number_of_channels=1,
    z_resolution=10
)  # Registers: ogaz00

# Get registered areas
motor_areas = feagi_agent.brain_output.get_registered_cortical_areas()
print(motor_areas)  # ['ogaz00', 'omot00']

# Connect with dynamic registration
await feagi_agent.feagi.connect_via_zmq(
    "tcp://localhost", 
    (128, 128, 3),
    motor_cortical_areas=motor_areas,  # ✅ Dynamic
    registration_port=30001,
    sensory_port=5558
)
```

## Registration Payload

The generated payload now includes all registered motor areas:

```json
{
  "method": "POST",
  "path": "/v1/agent/register",
  "body": {
    "agent_id": "autonomous_robot",
    "agent_type": "both",
    "capabilities": {
      "motor": {
        "modality": "wheel_motors",
        "output_count": 2,
        "source_cortical_areas": ["omot00", "ogaz00"]
      }
    }
  }
}
```

## Backward Compatibility

If no motor areas are registered or `motor_cortical_areas` is not provided:
- Defaults to `["omot00"]` for backward compatibility
- Existing code continues to work without modification

## Benefits

1. **Accurate Registration**: FEAGI knows exactly which motor areas the agent uses
2. **Extensible**: Supports any motor device type without code changes
3. **Type Safety**: Motor area names are generated from validated device registrations
4. **Self-Documenting**: Registration payload reflects actual agent capabilities

## Implementation Files

- `feagi_connector_2/cache/devices/motor_device_types.py` - Registration callbacks
- `feagi_connector_2/cache/motors_proxy.py` - Tracking and mapping
- `feagi_connector_2/feagi_interface.py` - Connection API update
- `feagi_connector_2/transport_interface/zmq.py` - Dynamic payload generation
- `examples/example_2_2.py` - Usage example

## Future Enhancements

1. **Vision tracking**: Apply same pattern to sensory devices
2. **Validation**: Verify cortical areas against brain genome
3. **Auto-discovery**: Query FEAGI for available cortical areas
4. **Configuration export**: Save registered capabilities to config file

## Migration Guide

### For Existing Code

No changes required - default behavior is preserved.

### For New Code

```python
# Old way (still works):
await feagi_agent.feagi.connect_via_zmq("tcp://localhost", (128, 128, 3))

# New way (recommended):
motor_areas = feagi_agent.brain_output.get_registered_cortical_areas()
await feagi_agent.feagi.connect_via_zmq(
    "tcp://localhost", 
    (128, 128, 3),
    motor_cortical_areas=motor_areas
)
```

## Testing

Run `example_2_2.py` to verify:
1. Both `omot00` and `ogaz00` appear in registration
2. FEAGI sends motor commands to both areas
3. Connector correctly decodes and displays both

## Related Documentation

- [API Migration Guide](API_MIGRATION_GUIDE.md)
- [Architecture Overview](Architecture.md)
- [Error Handling Improvements](../ERROR_HANDLING_IMPROVEMENTS.md)


