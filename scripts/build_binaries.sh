#!/bin/bash
# Build FEAGI binaries for all platforms and place in feagi/bin/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FEAGI_ROOT="$(cd "$SDK_ROOT/../feagi" && pwd)"

echo "=========================================="
echo "Building FEAGI Binaries for Python SDK"
echo "=========================================="
echo ""
echo "SDK Root: $SDK_ROOT"
echo "FEAGI Root: $FEAGI_ROOT"
echo ""

# Create bin directory structure
mkdir -p "$SDK_ROOT/feagi/bin/linux-x86_64"
mkdir -p "$SDK_ROOT/feagi/bin/linux-aarch64"
mkdir -p "$SDK_ROOT/feagi/bin/darwin-x86_64"
mkdir -p "$SDK_ROOT/feagi/bin/darwin-aarch64"
mkdir -p "$SDK_ROOT/feagi/bin/windows-x86_64"

cd "$FEAGI_ROOT"

# Detect current platform
OS=$(uname -s)
ARCH=$(uname -m)

echo "Current platform: $OS $ARCH"
echo ""

# Build for current platform (native)
echo "🔨 Building for native platform..."
cargo build --release

# Copy to appropriate directory
if [[ "$OS" == "Linux" ]]; then
    if [[ "$ARCH" == "x86_64" ]]; then
        cp target/release/feagi "$SDK_ROOT/feagi/bin/linux-x86_64/feagi"
        chmod +x "$SDK_ROOT/feagi/bin/linux-x86_64/feagi"
        echo "✅ Built: linux-x86_64/feagi"
    elif [[ "$ARCH" == "aarch64" ]]; then
        cp target/release/feagi "$SDK_ROOT/feagi/bin/linux-aarch64/feagi"
        chmod +x "$SDK_ROOT/feagi/bin/linux-aarch64/feagi"
        echo "✅ Built: linux-aarch64/feagi"
    fi
elif [[ "$OS" == "Darwin" ]]; then
    if [[ "$ARCH" == "x86_64" ]]; then
        cp target/release/feagi "$SDK_ROOT/feagi/bin/darwin-x86_64/feagi"
        chmod +x "$SDK_ROOT/feagi/bin/darwin-x86_64/feagi"
        echo "✅ Built: darwin-x86_64/feagi"
    elif [[ "$ARCH" == "arm64" ]]; then
        cp target/release/feagi "$SDK_ROOT/feagi/bin/darwin-aarch64/feagi"
        chmod +x "$SDK_ROOT/feagi/bin/darwin-aarch64/feagi"
        echo "✅ Built: darwin-aarch64/feagi"
    fi
fi

echo ""
echo "=========================================="
echo "✅ Build Complete!"
echo "=========================================="
echo ""
echo "Binaries created in: $SDK_ROOT/feagi/bin/"
ls -lh "$SDK_ROOT/feagi/bin/"*/*
echo ""
echo "⚠️  Note: Cross-platform builds require additional setup"
echo "    See BUNDLED_BINARIES.md for details"
echo ""







