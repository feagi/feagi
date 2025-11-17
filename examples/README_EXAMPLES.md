# FEAGI Connector Examples

This directory contains examples demonstrating the next-generation FEAGI connector (`feagi_connector_2`).

## Quick Start

### 1. Activate the Virtual Environment

```bash
cd /Users/nadji/code/FEAGI-2.0/feagi-connector/examples
source activate.sh
# or
source example_venv/bin/activate
```

### 2. Choose Your Example

We provide three versions of the same example for different use cases:

## Example Files

### 📄 `example_2_2.py` - Production Version
**Use when:** FEAGI is running and you want clean, minimal code

**Features:**
- Minimal code, no debug output
- Assumes FEAGI is running on localhost:30000
- Will hang if FEAGI is not available

**Run:**
```bash
python example_2_2.py
```

**⚠️ Note:** This version will hang with no output if FEAGI is not running. Use the debug version instead if you're not sure.

---

### 🔍 `example_2_2_debug.py` - Debug Version ⭐ RECOMMENDED
**Use when:** Testing connectivity or troubleshooting

**Features:**
- ✅ Progress output at each step
- ✅ Connection timeout (5 seconds)
- ✅ Clear error messages
- ✅ Helpful troubleshooting suggestions
- ✅ Shows exactly where problems occur

**Run:**
```bash
python example_2_2_debug.py
```

**Example output:**
```
============================================================
FEAGI Agent Example - Debug Version
============================================================

[1/6] Setting up image properties: (128, 128, 3)
      ✓ Image properties created

[2/6] Creating FEAGI agent...
      ✓ Agent created
...
```

---

### 🧪 `example_2_2_standalone.py` - Standalone Test Version
**Use when:** Testing agent functionality without FEAGI

**Features:**
- ✅ No FEAGI connection required
- ✅ Tests all local functionality
- ✅ Validates agent, cache, and image processing
- ✅ Multiple test cases
- ✅ Great for development and debugging

**Run:**
```bash
python example_2_2_standalone.py
```

**Example output:**
```
============================================================
FEAGI Agent Standalone Test
Testing agent functionality without FEAGI connection
============================================================

[TEST 1] Creating FEAGI agent...
         ✓ Agent created successfully
...
```

---

## What Each Example Does

All three examples demonstrate the same workflow:

1. **Create an agent** - Initialize `FeagiAgent`
2. **Configure camera** - Set up image properties (128x128 RGB)
3. **Register camera** - Register the camera with the agent
4. **Connect to FEAGI** - Establish ZMQ connection (except standalone)
5. **Create image** - Generate a test image
6. **Send to FEAGI** - Push image data through the connector

## Common Workflows

### First Time Setup
```bash
# 1. Test that everything is installed correctly
python example_2_2_standalone.py

# 2. Start FEAGI (in another terminal)
cd /Users/nadji/code/FEAGI-2.0/feagi-core
cargo run --release

# 3. Test connection to FEAGI
python example_2_2_debug.py

# 4. If successful, use the production version
python example_2_2.py
```

### Development Workflow
```bash
# Test your changes without FEAGI
python example_2_2_standalone.py

# Test with FEAGI when ready
python example_2_2_debug.py
```

### Debugging Connection Issues
```bash
# Use debug version to diagnose
python example_2_2_debug.py

# See troubleshooting guide
cat TROUBLESHOOTING.md
```

## Configuration

The examples connect to FEAGI using these default settings:

- **Host:** `tcp://localhost`
- **Registration Port:** `30000`
- **Brain Input Port:** `30001`
- **Brain Output Port:** `30002`

To change these, edit the `connect_via_zmq()` call in the example:

```python
registration_response = await feagi_agent.feagi.connect_via_zmq(
    "tcp://YOUR_HOST",           # Change host
    input_image_resolution, 
    registration_port=YOUR_PORT, # Change ports
    brain_input_port=30001,
    brain_output_port=30002
)
```

## Image Configuration

The examples use a 128x128 RGB image:

```python
input_image_resolution = (128, 128, 3)  # Width, Height, Channels
```

To change:
- **Resolution:** Modify the tuple (e.g., `(256, 256, 3)` for 256x256)
- **Grayscale:** Use 1 channel (e.g., `(128, 128, 1)`)
- **Color Space:** Change `ColorSpace.Linear` to other options
- **Layout:** Change between `RGB`, `BGR`, `RGBA`, etc.

## Troubleshooting

### Script Hangs With No Output
➡️ **Solution:** Use `example_2_2_debug.py` instead. See [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

### Connection Timeout
```
✗ CONNECTION FAILED: Timeout
```
➡️ **Solution:** 
1. Check FEAGI is running: `lsof -i :30000`
2. Start FEAGI if needed
3. Check firewall settings

### Module Not Found
```
ModuleNotFoundError: No module named 'feagi_connector_2'
```
➡️ **Solution:**
1. Activate virtual environment: `source activate.sh`
2. Reinstall: `cd .. && pip install -e . --force-reinstall --no-deps`

### Import Error for feagi_rust_py_libs
```
ModuleNotFoundError: No module named 'feagi_rust_py_libs'
```
➡️ **Solution:**
```bash
cd /Users/nadji/code/FEAGI-2.0/feagi-rust-py-libs
source ../feagi-connector/examples/example_venv/bin/activate
maturin develop --release
```

## Additional Resources

- **Setup Guide:** [SETUP.md](SETUP.md)
- **Troubleshooting:** [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **Main README:** [../README.md](../README.md)

## Requirements

All requirements are pre-installed in the virtual environment:
- `numpy >= 1.20.0`
- `pyzmq >= 24.0.0`
- `aiohttp >= 3.9.0`
- `toml >= 0.10.2`
- `feagi_rust_py_libs` (built from source)
- `feagi_connector` (editable install)

See [requirements.txt](requirements.txt) for full list.

