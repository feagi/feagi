# Bundling FEAGI Binaries with Python SDK

This document explains how to build and bundle pre-compiled FEAGI binaries with the Python SDK, allowing users to install FEAGI via `pip install feagi` without needing Rust or access to the source code.

## Why Bundle Binaries?

**Benefits:**
- ✅ Users don't need Rust toolchain
- ✅ Rust source code stays private
- ✅ Single command install: `pip install feagi`
- ✅ No compilation needed
- ✅ Faster installation

**Trade-offs:**
- ⚠️ Larger package size (~50-100MB per platform)
- ⚠️ Need to build for multiple platforms
- ⚠️ Platform-specific wheels required

## Directory Structure

```
feagi-python-sdk/
├── feagi/
│   ├── bin/                           # Bundled binaries
│   │   ├── linux-x86_64/
│   │   │   └── feagi                  # Linux x86_64 binary
│   │   ├── linux-aarch64/
│   │   │   └── feagi                  # Linux ARM64 binary
│   │   ├── darwin-x86_64/
│   │   │   └── feagi                  # macOS Intel binary
│   │   ├── darwin-aarch64/
│   │   │   └── feagi                  # macOS Apple Silicon binary
│   │   └── windows-x86_64/
│   │       └── feagi.exe              # Windows x86_64 binary
│   ├── engine/
│   │   └── manager.py                 # Auto-detects bundled binary
│   └── ...
├── pyproject.toml                      # Includes binaries in package
├── MANIFEST.in                         # Ensures binaries are distributed
└── scripts/
    └── build_binaries.sh               # Helper script
```

## Building Binaries

### Option 1: Native Build (Easiest)

Build for your current platform only:

```bash
cd feagi-python-sdk
./scripts/build_binaries.sh
```

This will:
1. Build FEAGI in release mode
2. Copy binary to appropriate `feagi/bin/` directory
3. Set executable permissions

### Option 2: Cross-Platform Build (GitHub Actions)

Use GitHub Actions to build for all platforms automatically.

**Create**: `.github/workflows/build-binaries.yml`

```yaml
name: Build FEAGI Binaries for Python SDK

on:
  workflow_dispatch:
  push:
    branches:
      - staging
    paths:
      - 'feagi/**'

jobs:
  build-linux-x86_64:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions-rs/toolchain@v1
        with:
          toolchain: stable
      - name: Build FEAGI
        run: |
          cd feagi
          cargo build --release
      - name: Upload binary
        uses: actions/upload-artifact@v4
        with:
          name: feagi-linux-x86_64
          path: feagi/target/release/feagi

  build-macos-aarch64:
    runs-on: macos-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions-rs/toolchain@v1
        with:
          toolchain: stable
      - name: Build FEAGI
        run: |
          cd feagi
          cargo build --release
      - name: Upload binary
        uses: actions/upload-artifact@v4
        with:
          name: feagi-darwin-aarch64
          path: feagi/target/release/feagi

  # Add more jobs for other platforms...
  
  package:
    needs: [build-linux-x86_64, build-macos-aarch64]
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Download all binaries
        uses: actions/download-artifact@v4
        with:
          path: binaries
      
      - name: Organize binaries
        run: |
          mkdir -p feagi-python-sdk/feagi/bin/linux-x86_64
          mkdir -p feagi-python-sdk/feagi/bin/darwin-aarch64
          cp binaries/feagi-linux-x86_64/feagi feagi-python-sdk/feagi/bin/linux-x86_64/
          cp binaries/feagi-darwin-aarch64/feagi feagi-python-sdk/feagi/bin/darwin-aarch64/
          chmod +x feagi-python-sdk/feagi/bin/*/feagi
      
      - name: Build Python package
        run: |
          cd feagi-python-sdk
          pip install build
          python -m build
      
      - name: Upload package
        uses: actions/upload-artifact@v4
        with:
          name: feagi-python-package
          path: feagi-python-sdk/dist/
```

### Option 3: Cross-Compilation (Advanced)

Use Rust cross-compilation tools:

```bash
# Install cross
cargo install cross

# Build for Linux ARM64 (from any platform)
cd feagi
cross build --target aarch64-unknown-linux-gnu --release
cp target/aarch64-unknown-linux-gnu/release/feagi ../feagi-python-sdk/feagi/bin/linux-aarch64/

# Build for Windows (from Linux/macOS)
cross build --target x86_64-pc-windows-gnu --release
cp target/x86_64-pc-windows-gnu/release/feagi.exe ../feagi-python-sdk/feagi/bin/windows-x86_64/
```

## Publishing Workflow

### 1. Build Binaries
```bash
# Automated (GitHub Actions) - builds all platforms
# OR
# Manual (current platform only)
./scripts/build_binaries.sh
```

### 2. Verify Binaries
```bash
ls -lh feagi/bin/*/
# Should show binaries for all platforms
```

### 3. Test Locally
```bash
# Install in editable mode
pip install -e .

# Test
python -c "from feagi.engine import FeagiEngine; print('✅ Works!')"
```

### 4. Build Package
```bash
python -m build
```

### 5. Publish to Test PyPI
```bash
python -m twine upload --repository testpypi dist/*
```

### 6. Test Installation
```bash
# On each platform
pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ feagi
```

## Binary Detection Priority

`FeagiEngine` looks for binaries in this order:

1. **Bundled binary** (in package) ← Highest priority
2. Development build (local `feagi/target/release/`)
3. System PATH (`which feagi`)
4. Common install locations (`/usr/local/bin/`, `~/.cargo/bin/`)

This ensures:
- ✅ Package works out-of-the-box for users
- ✅ Developers can still use local builds
- ✅ System installations still work

## Package Sizes

Approximate sizes per platform:

| Platform | Binary Size | Notes |
|----------|-------------|-------|
| Linux x86_64 | ~20-30 MB | With debug symbols stripped |
| Linux aarch64 | ~20-30 MB | ARM64 |
| macOS x86_64 | ~25-35 MB | Intel Mac |
| macOS aarch64 | ~25-35 MB | Apple Silicon |
| Windows x86_64 | ~25-35 MB | |

**Total wheel size**: ~100-150 MB (all platforms)

### Reducing Size

Strip debug symbols:
```bash
cd feagi
cargo build --release
strip target/release/feagi  # Linux/macOS
```

Use UPX compression (optional):
```bash
upx --best target/release/feagi
# Can reduce size by 50-70%
```

## Platform-Specific Wheels

Create separate wheels for each platform:

```bash
# Build wheel for current platform only
python -m build --wheel

# Results in platform-specific wheel:
# dist/feagi-2.0.0-py3-none-linux_x86_64.whl
# dist/feagi-2.0.0-py3-none-macosx_11_0_arm64.whl
# etc.
```

Users automatically get correct wheel for their platform via pip.

## Security Considerations

### Code Signing (Recommended)

**macOS**:
```bash
codesign -s "Developer ID Application: Your Name" feagi/bin/darwin-*/feagi
```

**Windows**:
```bash
signtool sign /f certificate.pfx /p password feagi/bin/windows-x86_64/feagi.exe
```

### Checksum Verification

Generate checksums:
```bash
cd feagi/bin
sha256sum */* > CHECKSUMS.txt
```

Include in package for verification.

## Troubleshotics

### "Binary not executable"
```bash
chmod +x feagi/bin/*/feagi
```

### "Binary not found"
Check package includes binaries:
```bash
python -c "import feagi; print(feagi.__file__)"
# Should show: .../site-packages/feagi/__init__.py
# Check: .../site-packages/feagi/bin/ exists
```

### "Wrong architecture"
Ensure binary was built for target platform:
```bash
file feagi/bin/linux-x86_64/feagi
# Should show: ELF 64-bit LSB executable, x86-64
```

## Future: Separate Binary Package

For even cleaner separation, consider:
```bash
pip install feagi              # Core SDK
pip install feagi-runtime      # Binaries only
```

This allows:
- Lighter core package
- Optional binary installation
- User choice to build from source

## Summary

**Current State**: Code updated to support bundled binaries ✅

**To Publish**:
1. Build binaries for all platforms (GitHub Actions)
2. Place in `feagi/bin/`
3. Build Python package: `python -m build`
4. Publish: `twine upload dist/*`

**Users Get**:
- Single command: `pip install feagi`
- No Rust needed
- Works immediately
- Your source code stays private ✅






