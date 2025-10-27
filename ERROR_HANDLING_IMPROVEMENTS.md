# FEAGI Connector: Enhanced Error Handling

## What Was Improved ✅

The `feagi-connector` now provides **detailed, actionable error messages** instead of generic "Unknown error" messages.

### Before (Useless ❌)
```
ERROR - Failed to connect: Connection failed: Registration failed: Unknown error
```

### After (Actionable ✅)
```
ERROR - ❌ Registration failed with unclear error from Rust SDK

Common causes:
  1. FEAGI's Rust PNS is not running
     Check FEAGI logs for: '🦀 [ZMQ-REGISTRATION] Listening on...'
  2. Port mismatch (FEAGI 2.0 uses port 5563, not 30001)
     Your config: registration_port = ?
     FEAGI listening: Check with 'lsof -i :5563'
  3. ZMQ socket state issue (restart FEAGI)
  4. Firewall blocking connection

RuntimeError: Registration failed: ... Check FEAGI is running with Rust PNS on port 5563. See logs above for troubleshooting steps.
```

## Enhancements Added

### 1. Pre-Flight Network Check

**Before connection attempt**, the connector now:
- ✅ Checks if FEAGI is listening on the registration port
- ✅ Provides immediate feedback if FEAGI is unreachable
- ✅ Suggests troubleshooting commands (`lsof -i :5563`)

```python
# Socket check before attempting registration
sock.connect_ex((host, port))
if result != 0:
    logger.error(f"❌ Cannot reach FEAGI at {host}:{port}")
    logger.error("Troubleshooting:")
    logger.error("  1. Check if FEAGI is running")
    logger.error("  2. Verify FEAGI is listening on port...")
```

### 2. Enhanced Registration Error Messages

**Registration failures** now include:
- ✅ Specific error type identification
- ✅ Common causes checklist
- ✅ FEAGI log patterns to look for
- ✅ Port configuration verification
- ✅ Actionable next steps

```python
if "unknown error" in error_str or "registration failed" in error_str:
    logger.error("❌ Registration failed with unclear error from Rust SDK")
    logger.error("")
    logger.error("Common causes:")
    logger.error("  1. FEAGI's Rust PNS is not running")
    logger.error("     Check FEAGI logs for: '🦀 [ZMQ-REGISTRATION] Listening on...'")
    # ... more helpful context
```

### 3. Detailed Debug Logging

**DEBUG mode** now shows:
- ✅ Configuration values being used
- ✅ Connection attempt progress
- ✅ Registration endpoint details
- ✅ Step-by-step connection flow

```python
logger.debug(f"  FEAGI host: {self._feagi_host}")
logger.debug(f"  Registration port: {registration_port}")
logger.debug(f"  Sensory port: {self._sensory_port}")
logger.debug("Creating Rust-backed agent client...")
logger.debug("Attempting registration with FEAGI...")
```

### 4. Success Confirmation Messages

**Successful connections** now show:
- ✅ Clear success indicator
- ✅ Connection details summary
- ✅ Heartbeat configuration
- ✅ Binary XYZP socket status

```python
logger.info(f"✓ Connected and registered as: {self.agent_id}")
logger.info(f"  Registration: {host}:{registration_port}")
logger.info(f"  Sensory data: {host}:{sensory_port}")
logger.info(f"  Heartbeat: {heartbeat_interval}s")
```

### 5. Unexpected Error Reporting

**Unexpected errors** include:
- ✅ Error type identification
- ✅ Full context for bug reporting
- ✅ Agent configuration details
- ✅ Instructions for reporting issues

```python
logger.error(f"❌ Unexpected error during connection: {e}")
logger.error(f"   Error type: {type(e).__name__}")
logger.error("")
logger.error("Please report this error with the following information:")
logger.error(f"  - Agent ID: {self.agent_id}")
logger.error(f"  - FEAGI host: {self._feagi_host}")
logger.error(f"  - Registration port: {registration_port}")
```

## Usage Example

### Running Agent with Debug Logging

```bash
python agent.py --log-level DEBUG
```

**Output** (example of network failure):
```
INFO - Connecting to FEAGI as: video-agent-1
DEBUG -   FEAGI host: 127.0.0.1
DEBUG -   Registration port: tcp://127.0.0.1:5563
DEBUG -   Sensory port: 5558
ERROR - ❌ Cannot reach FEAGI at 127.0.0.1:5563

Troubleshooting:
  1. Check if FEAGI is running
  2. Verify FEAGI is listening on port 5563
     Run: lsof -i :5563
  3. Check your config.toml has the correct host and port
  4. For FEAGI 2.0, registration_port should be 5563 (Rust PNS)

ERROR - Failed to connect to FEAGI: Connection failed: FEAGI not reachable at 127.0.0.1:5563. Is FEAGI running? Check 'lsof -i :5563'
```

**Output** (example of registration failure):
```
INFO - Connecting to FEAGI as: video-agent-1
DEBUG -   FEAGI host: 127.0.0.1
DEBUG -   Registration port: tcp://127.0.0.1:5563
DEBUG -   Sensory port: 5558
DEBUG - ✓ FEAGI is listening on 127.0.0.1:5563
DEBUG - Creating Rust-backed agent client...
DEBUG - Attempting registration with FEAGI...
ERROR - ❌ Registration failed with unclear error from Rust SDK

Common causes:
  1. FEAGI's Rust PNS is not running
     Check FEAGI logs for: '🦀 [ZMQ-REGISTRATION] Listening on...'
  2. Port mismatch (FEAGI 2.0 uses port 5563, not 30001)
     Your config: registration_port = ?
     FEAGI listening: Check with 'lsof -i :5563'
  3. ZMQ socket state issue (restart FEAGI)
  4. Firewall blocking connection
```

## Error Categories

### Category 1: Network Unreachable
**Symptom**: Cannot reach FEAGI at host:port  
**Action**: 
1. Check FEAGI is running
2. Verify port with `lsof -i :5563`
3. Check firewall rules
4. Verify config.toml host/port values

### Category 2: Registration Failed
**Symptom**: Registration failed with unclear error  
**Action**:
1. Check FEAGI logs for Rust PNS startup
2. Verify port 5563 (not 30001)
3. Restart FEAGI to clear ZMQ state
4. Check FEAGI configuration

### Category 3: ZMQ Socket Issues
**Symptom**: Failed to create direct ZMQ socket  
**Action**:
- Agent will fall back to JSON method (less efficient)
- Check sensory port configuration
- Verify ZMQ library installation

### Category 4: Unexpected Errors
**Symptom**: Any error not matching above patterns  
**Action**:
- Report to FEAGI team with full error details
- Include agent ID, host, port, error type

## Architecture Benefits

### FEAGI 2.0 Compliance
- ✅ **Fail-Fast**: Immediate, clear failure messages
- ✅ **No Silent Failures**: All errors are logged with context
- ✅ **Actionable**: Users know exactly what to do next
- ✅ **Debuggable**: DEBUG mode shows full connection flow

### Developer Experience
- ✅ **Reduces frustration**: No more "Unknown error"
- ✅ **Saves time**: Clear troubleshooting steps
- ✅ **Self-service**: Most issues can be resolved without support
- ✅ **Better bug reports**: Full context for unexpected errors

## Testing Error Handling

### Test 1: FEAGI Not Running
```bash
# Stop FEAGI first
python agent.py
# Expected: Clear message that FEAGI is unreachable
```

### Test 2: Wrong Port
```toml
# In config.toml
[feagi]
registration_port = 30001  # Wrong port (legacy)
```
```bash
python agent.py --log-level DEBUG
# Expected: Port mismatch warning, connection failure
```

### Test 3: Successful Connection
```bash
# With FEAGI running on correct ports
python agent.py --log-level DEBUG
# Expected: Clear success messages with connection details
```

## Files Modified

- **`feagi-connector/feagi_connector/agent_client.py`**
  - Enhanced `connect()` method with:
    - Pre-flight network check
    - Enhanced error messages
    - Debug logging
    - Success confirmations

## Future Enhancements

Potential improvements for future versions:

1. **Rust SDK Error Types**: Return structured error types instead of generic strings
2. **Network Diagnostics**: Ping test, traceroute for remote FEAGI
3. **Configuration Validation**: Pre-validate all config before connection attempt
4. **Auto-Recovery**: Automatic reconnection with exponential backoff
5. **Health Check Endpoint**: Query FEAGI status before registration

---

**Status**: ✅ Implemented and ready for testing
**Impact**: Dramatically improved debugging experience for agent developers

