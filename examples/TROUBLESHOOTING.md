# Troubleshooting Guide for example_2_2.py

## Issue: Script Hangs With No Output

### Problem
When you run `example_2_2.py`, the script appears to hang with no output and doesn't complete.

### Root Cause
The script is waiting indefinitely for FEAGI to respond to a connection request. If FEAGI is not running or not accessible, the ZeroMQ socket will block forever at the `recv()` call.

### Solution

#### Option 1: Start FEAGI First (Recommended)
Before running the example, make sure FEAGI is running and accessible:

1. **Check if FEAGI is running:**
   ```bash
   # Check if ports are open
   lsof -i :30000  # Registration port
   lsof -i :30001  # Brain input port
   lsof -i :30002  # Brain output port
   ```

2. **Start FEAGI** (if not running):
   ```bash
   # Navigate to FEAGI core directory
   cd /Users/nadji/code/FEAGI-2.0/feagi-core
   
   # Start FEAGI (adjust command as needed for your setup)
   cargo run --release
   ```

3. **Run the example:**
   ```bash
   cd /Users/nadji/code/FEAGI-2.0/feagi-connector/examples
   source example_venv/bin/activate
   python example_2_2.py
   ```

#### Option 2: Use the Debug Version
Use the debug version that has timeouts and helpful error messages:

```bash
cd /Users/nadji/code/FEAGI-2.0/feagi-connector/examples
source example_venv/bin/activate
python example_2_2_debug.py
```

This will:
- Show progress at each step
- Timeout after 5 seconds if FEAGI doesn't respond
- Provide clear error messages
- Suggest next steps

#### Option 3: Use the Standalone Test Version
Test the agent functionality without requiring FEAGI:

```bash
cd /Users/nadji/code/FEAGI-2.0/feagi-connector/examples
source example_venv/bin/activate
python example_2_2_standalone.py
```

This version:
- Tests all agent functionality locally
- Doesn't require FEAGI to be running
- Useful for development and debugging

### Common Issues

#### Port Conflicts
If you get a "port already in use" error:
```bash
# Find what's using the port
lsof -i :30000

# Kill the process if needed
kill -9 <PID>
```

#### Connection Refused
If FEAGI is running but connection is refused:
- Check firewall settings
- Verify FEAGI configuration (`feagi_configuration.toml`)
- Ensure FEAGI is bound to the correct interface (0.0.0.0 or localhost)

#### Different Host/Port
If FEAGI is running on a different host or port, modify the connection:

```python
# In example_2_2.py, line 25, change:
registration_response = await feagi_agent.feagi.connect_via_zmq(
    "tcp://YOUR_HOST",  # Change this
    input_image_resolution, 
    registration_port=YOUR_PORT,  # And this
    brain_input_port=YOUR_INPUT_PORT,
    brain_output_port=YOUR_OUTPUT_PORT
)
```

### Debugging Checklist

- [ ] Virtual environment is activated
- [ ] All packages are installed (`pip list | grep feagi`)
- [ ] FEAGI is running
- [ ] FEAGI ports are accessible
- [ ] No firewall blocking connections
- [ ] Correct host and port configuration
- [ ] Network connectivity (if using remote FEAGI)

### Getting Help

If you're still having issues:
1. Run the debug version and capture the output
2. Check FEAGI logs for connection attempts
3. Verify network connectivity: `nc -zv localhost 30000`
4. Check system logs: `dmesg` or Console.app (macOS)

