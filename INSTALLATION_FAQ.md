# FEAGI Installation FAQ

Common questions about installing FEAGI from PyPI.

## Do I need to install requirements separately?

**No!** When you run `pip install feagi`, all dependencies are automatically installed.

### What Pip Does Automatically

```bash
pip install feagi
```

Behind the scenes:
1. ✅ Downloads pre-built wheel with compiled Rust
2. ✅ Reads dependency list from package metadata
3. ✅ Installs FastAPI, Uvicorn, Pydantic
4. ✅ Installs NumPy, SciPy, PyTorch
5. ✅ Installs PyZMQ, LZ4, psutil
6. ✅ Installs all 20+ dependencies
7. ✅ Ready to use!

### You DON'T Need To

❌ `pip install -r requirements.txt`  
❌ Manually install numpy, torch, fastapi, etc.  
❌ Install each dependency one by one  

### You ONLY Need To

✅ `pip install feagi`

That's it! Everything is automatic.

## Do I need Rust installed?

**No!** Pre-built wheels include compiled Rust extensions.

### Why Rust is Pre-compiled

- Rust code compiled during CI/CD (once)
- Distributed as binary in the wheel
- Users get instant installation
- No compilation on user machines

### When You DO Need Rust

Only if you're:
- Developing FEAGI (modifying Rust code)
- Using unsupported platform (Raspberry Pi, Alpine Linux, etc.)
- Installing from source: `pip install feagi --no-binary feagi`

## What gets installed?

### Single Command
```bash
pip install feagi
```

### Installs All Of This

**Core Package:**
- FEAGI Python code
- Pre-compiled Rust extensions (feagi_rust.so/.dylib/.pyd)
- Configuration files

**Automatic Dependencies (~500MB):**
- **Web Framework**: FastAPI, Uvicorn
- **Data Science**: NumPy, SciPy, PyTorch
- **Communication**: PyZMQ (ZeroMQ bindings)
- **Compression**: LZ4
- **System**: psutil
- **Validation**: Pydantic
- **Security**: python-jose, passlib
- **And more...**

## Installation Size

| Component | Size | Installation Time |
|-----------|------|-------------------|
| FEAGI wheel (with Rust) | 5-8 MB | 2-5 seconds |
| All Python dependencies | ~500 MB | 20-40 seconds |
| **Total** | **~505-510 MB** | **25-45 seconds** |

Compare to building from source:
- Requires Rust toolchain: 1.5 GB
- Compilation time: 5-10 minutes
- Total time: 10-15 minutes

## Platform-Specific Notes

### Linux
```bash
# Everything automatic
pip install feagi
```

### macOS (Intel or Apple Silicon)
```bash
# Everything automatic (correct wheel detected automatically)
pip install feagi
```

### Windows
```bash
# Everything automatic
pip install feagi
```

### Docker
```dockerfile
FROM python:3.10-slim
RUN pip install feagi  # All dependencies included
CMD ["python", "-m", "feagi.main"]
```

## Troubleshooting

### "Could not find a version that satisfies the requirement"

Your platform might not have a pre-built wheel. Check:
- Python version (must be 3.8-3.12)
- Platform (Linux x86_64, macOS, Windows x64)

### Large Download Size

Normal! PyTorch alone is ~200-300 MB. Total ~500 MB with all dependencies.

### "No module named 'feagi'"

Did you activate your virtual environment?
```bash
source venv/bin/activate  # Linux/macOS
venv\Scripts\activate     # Windows
```

### ImportError for Rust Extensions

```bash
# Verify Rust extensions loaded
python -c "from feagi import feagi_rust; print('OK')"
```

If this fails, your wheel might be corrupted. Reinstall:
```bash
pip uninstall feagi
pip install --no-cache-dir feagi
```

## Comparison: requirements.txt vs PyPI

### Old Way (Manual Dependencies)
```bash
# Step 1: Install dependencies manually
pip install -r requirements.txt

# Step 2: Build Rust (requires Rust toolchain)
pip install -e .

# Step 3: Hope everything works
python -m feagi.main
```

### New Way (PyPI)
```bash
# One command, everything automatic
pip install feagi

# Just works
python -m feagi.main
```

## Virtual Environment Best Practice

Always use a virtual environment:

### Why?
- Isolates FEAGI dependencies
- Prevents conflicts with other projects
- Easy to recreate if issues occur
- Clean uninstall: just delete venv folder

### How?
```bash
# Create
python3 -m venv feagi-venv

# Activate
source feagi-venv/bin/activate  # Linux/macOS
feagi-venv\Scripts\activate     # Windows

# Install
pip install feagi

# Use
python -m feagi.main

# Deactivate when done
deactivate
```

## Developer Installation

If you're developing FEAGI itself:

```bash
# Clone repository
git clone https://github.com/neuraville/feagi.git
cd feagi/feagi_core

# Create venv
python3 -m venv venv
source venv/bin/activate

# Install in development mode (requires Rust)
pip install -e .

# Rust extensions compile automatically
# Dependencies installed automatically
```

## Summary

| Question | Answer |
|----------|--------|
| Do I need requirements.txt? | ❌ No |
| Do I need Rust? | ❌ No (pre-compiled) |
| Do I need to manually install deps? | ❌ No (automatic) |
| What command installs everything? | ✅ `pip install feagi` |
| How long does it take? | ✅ 25-45 seconds |
| What platforms are supported? | ✅ Linux, macOS, Windows |
| What Python versions? | ✅ 3.8, 3.9, 3.10, 3.11, 3.12 |

## Still Have Questions?

- Check the [main README](README.md)
- Review [troubleshooting guide](README.md#troubleshooting)
- File an issue on GitHub

