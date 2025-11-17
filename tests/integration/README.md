# FEAGI SDK Integration Tests

This directory contains system-level integration tests that verify the FEAGI SDK works end-to-end with a running FEAGI instance.

## Prerequisites

### 1. Install Test Dependencies

```bash
cd feagi-python-sdk
pip install -e ".[test]"
```

Required packages:
- `pytest` - Test framework
- `opencv-python` - Video frame reading
- `requests` - FEAGI REST API checks
- `feagi_rust_py_libs` - High-performance encoding (optional but recommended)

### 2. Start FEAGI

**Current (Manual):**
```bash
# Start FEAGI on localhost
# Default ports: 8000 (REST), 5558 (sensory), 5564 (motor)
```

**Future (Phase 2 - Automated):**
```python
from feagi.engine import FeagiEngine

engine = FeagiEngine()
engine.start()
engine.load_genome("genome.json")
```

### 3. Prepare Test Assets

Place test video file:
```bash
# Copy your test video to examples/
cp /path/to/vt_all.mov feagi-python-sdk/examples/
```

---

## Running Tests

### Run All Integration Tests

```bash
cd feagi-python-sdk
pytest tests/integration/ -v
```

### Run Specific Test

```bash
# Video streaming test
pytest tests/integration/test_video_to_feagi.py -v

# Or run standalone
python tests/integration/test_video_to_feagi.py
```

### Run with Coverage

```bash
pytest tests/integration/ -v --cov=feagi --cov-report=html
```

---

## Available Tests

### `test_video_to_feagi.py`

**Purpose:** End-to-end test of video streaming to FEAGI

**What it tests:**
- Video file reading (OpenCV)
- Camera input registration (new SDK)
- FEAGI connection via ZMQ
- Frame encoding (Rust-backed)
- Data transmission to FEAGI
- Performance metrics (FPS)

**Requirements:**
- FEAGI running on `localhost:8000`
- Video file at `examples/vt_all.mov`
- `feagi_rust_py_libs` installed

**Expected Output:**
```
📹 Opening video: examples/vt_all.mov
   Resolution: 1920x1080
   FPS: 30.0
   Total frames: 300

📥 Registering Camera input with FEAGI SDK 3.0...
   ✓ Camera registered

🔧 Configuring connection to FEAGI...
   Host: localhost
   Port: 5558

🔗 Connecting to FEAGI...
   ✓ Connected successfully!

🎬 Streaming frames to FEAGI...
   Frame 10/100 (10.2 fps)
   Frame 20/100 (10.5 fps)
   ...

📊 Test Results:
   Frames sent: 100/100
   Time elapsed: 9.52s
   Actual FPS: 10.5
   Status: ✅ PASS
```

---

## Test Configuration

### Default Configuration

```python
feagi_config = {
    "host": "localhost",
    "sensory_port": 5558,
    "motor_port": 5564,
    "rest_port": 8000,
}
```

### Override via Environment Variables

```bash
export FEAGI_HOST=192.168.1.100
export FEAGI_SENSORY_PORT=5558
export FEAGI_MOTOR_PORT=5564
export FEAGI_REST_PORT=8000

pytest tests/integration/ -v
```

---

## Troubleshooting

### "FEAGI is not running"

**Problem:** Test skipped because FEAGI is not accessible

**Solution:**
1. Start FEAGI manually
2. Verify FEAGI is listening on correct ports:
   ```bash
   curl http://localhost:8000/v1/feagi/status
   ```
3. Check firewall settings

### "Video file not found"

**Problem:** Test can't find `vt_all.mov`

**Solution:**
1. Place video file in `examples/` directory
2. Or update `video_path` fixture in test
3. Or use any `.mov`/`.mp4` file for testing

### "feagi_rust_py_libs not found"

**Problem:** Rust SDK not installed

**Solution:**
```bash
pip install feagi_rust_py_libs
```

Or build from source if needed.

### "Failed to connect to FEAGI"

**Problem:** Connection to FEAGI sensory port failed

**Solution:**
1. Verify FEAGI is running
2. Check port is correct (default: 5558)
3. Ensure no firewall blocking
4. Check FEAGI logs for errors

---

## Adding New Integration Tests

### Test Template

```python
"""
Integration Test: [Feature Name]

Requirements:
- FEAGI running
- [Other requirements]
"""

import pytest
from feagi.pns.inputs import [YourInput]
from feagi.pns import brain_input

class Test[FeatureName]:
    
    @pytest.fixture(scope="class")
    def feagi_config(self):
        return {
            "host": "localhost",
            "sensory_port": 5558,
        }
    
    def test_[feature](self, feagi_config):
        # Your test here
        pass
```

### Best Practices

1. **Check prerequisites** - Skip test if FEAGI not running
2. **Cleanup resources** - Always disconnect/close in `finally` block
3. **Meaningful assertions** - Verify actual behavior, not just "didn't crash"
4. **Performance metrics** - Log timing/throughput for benchmarking
5. **Clear output** - Print progress for long-running tests

---

## Future Tests (Roadmap)

- [ ] `test_motor_feedback.py` - Bidirectional communication
- [ ] `test_genome_loading.py` - Dynamic genome loading
- [ ] `test_multi_sensor.py` - Multiple simultaneous inputs
- [ ] `test_text_generation.py` - Language model integration
- [ ] `test_game_ai.py` - Game AI with state/action
- [ ] `test_performance.py` - Throughput and latency benchmarks
- [ ] `test_reconnection.py` - Connection resilience
- [ ] `test_engine_lifecycle.py` - Start/stop FEAGI (Phase 2)

---

## CI/CD Integration

### GitHub Actions (Example)

```yaml
name: Integration Tests

on: [push, pull_request]

jobs:
  integration-test:
    runs-on: ubuntu-latest
    
    services:
      feagi:
        image: neuraville/feagi:latest
        ports:
          - 8000:8000
          - 5558:5558
          - 5564:5564
    
    steps:
      - uses: actions/checkout@v3
      
      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      
      - name: Install dependencies
        run: |
          pip install -e ".[test]"
          pip install feagi_rust_py_libs
      
      - name: Run integration tests
        run: pytest tests/integration/ -v
```

---

## Questions?

- Check main README: `feagi-python-sdk/README.md`
- Review examples: `feagi-python-sdk/examples/`
- Ask on Discord: [link]
- Report issues: [GitHub Issues]

