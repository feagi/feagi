#!/bin/bash
# Copyright 2025 Neuraville Inc.
# Licensed under the Apache License, Version 2.0
#
# Test script to validate wheel building locally before pushing to CI

set -e

echo "🔧 FEAGI Wheel Build Test"
echo "=========================="
echo ""

# Check prerequisites
echo "Checking prerequisites..."

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

if ! command -v cargo &> /dev/null; then
    echo "❌ Rust/Cargo not found. Install from https://rustup.rs/"
    exit 1
fi

echo "✅ Python: $(python3 --version)"
echo "✅ Rust: $(rustc --version)"
echo ""

# Get to the right directory
cd "$(dirname "$0")/.."
PROJECT_ROOT=$(pwd)
echo "Project root: $PROJECT_ROOT"
echo ""

# Create a clean virtual environment
echo "Creating test virtual environment..."
rm -rf test-wheel-venv
python3 -m venv test-wheel-venv
source test-wheel-venv/bin/activate

# Upgrade build tools
echo ""
echo "Installing build tools..."
pip install --upgrade pip setuptools wheel setuptools-rust build

# Build the wheel
echo ""
echo "Building wheel (this will take 5-10 minutes)..."
python -m build --wheel

# Check the wheel was created
if [ ! -d "dist" ]; then
    echo "❌ dist/ directory not created"
    exit 1
fi

WHEEL_FILE=$(ls -t dist/*.whl | head -1)
if [ -z "$WHEEL_FILE" ]; then
    echo "❌ No wheel file found in dist/"
    exit 1
fi

echo ""
echo "✅ Wheel built successfully: $WHEEL_FILE"
echo "   Size: $(du -h "$WHEEL_FILE" | cut -f1)"

# Test installing the wheel
echo ""
echo "Testing wheel installation..."
pip install "$WHEEL_FILE"

# Verify the installation
echo ""
echo "Verifying installation..."
python -c "import feagi; print(f'✅ FEAGI version: {feagi.__version__}')" || {
    echo "❌ Failed to import feagi"
    exit 1
}

python -c "from feagi import feagi_rust; print('✅ Rust extensions loaded successfully')" || {
    echo "❌ Failed to import Rust extensions"
    exit 1
}

# Run a quick smoke test
echo ""
echo "Running smoke test..."
python -c "
from feagi import feagi_rust
print('✅ Rust module imported successfully')
print(f'   Available: {dir(feagi_rust)}')
"

# Cleanup
echo ""
echo "Cleaning up..."
deactivate
rm -rf test-wheel-venv

echo ""
echo "=========================================="
echo "✅ Wheel build and installation test PASSED"
echo "=========================================="
echo ""
echo "Your wheel is ready for distribution!"
echo "Wheel location: $WHEEL_FILE"
echo ""
echo "Next steps:"
echo "1. Test on different Python versions (3.8, 3.9, 3.10, 3.11, 3.12)"
echo "2. Test on different platforms (Linux, macOS, Windows)"
echo "3. When ready, create a GitHub release to trigger CI wheel builds"

