#!/bin/bash
# Build script for feagi-bdu Rust extension
# Phase 1: Synaptogenesis hot path migration

set -e  # Exit on error

echo "🦀 Building FEAGI BDU Rust Extension - Phase 1"
echo "================================================"

# Check for required tools
command -v cargo >/dev/null 2>&1 || { echo "❌ cargo not found. Install Rust: https://rustup.rs/"; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "❌ python3 not found"; exit 1; }

# Check for virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo "⚠️  No virtual environment detected!"
    echo "   Please activate your virtual environment first:"
    echo "   source .venv_feagi/bin/activate"
    exit 1
fi

echo "✓ Using virtual environment: $VIRTUAL_ENV"

# Check for maturin
if ! python3 -c "import maturin" 2>/dev/null; then
    echo "📦 Installing maturin in virtual environment..."
    pip install maturin
fi

echo ""
echo "1️⃣  Building Python extension with maturin..."
cd "$(dirname "$0")"/crates/feagi-bdu
maturin develop --release

echo ""
echo "2️⃣  Verifying Python import..."
python3 -c "
try:
    from feagi_bdu import py_syn_projector
    print('✅ Rust BDU successfully installed and importable')
    print('   Available functions: py_syn_projector, py_syn_projector_batch')
except ImportError as e:
    print(f'❌ Failed to import: {e}')
    exit(1)
"

echo ""
echo "3️⃣  Running quick performance test..."
python3 -c "
import time
from feagi_bdu import py_syn_projector

# Test projection (should be <10ms)
start = time.time()
result = py_syn_projector(
    'src', 'dst', 42,
    (128, 128, 3), (128, 128, 1),
    (64, 64, 1),
    None, None
)
elapsed = (time.time() - start) * 1000

print(f'✅ Single projection: {elapsed:.2f}ms')
print(f'   Result: {len(result)} positions')

# Quick batch test
neuron_ids = list(range(100))
locations = [(i % 128, i // 128, 0) for i in range(100)]

start = time.time()
from feagi_bdu import py_syn_projector_batch
results = py_syn_projector_batch(
    'src', 'dst', neuron_ids, locations,
    (128, 128, 3), (128, 128, 1),
    None, None
)
elapsed = (time.time() - start) * 1000

print(f'✅ Batch projection (100 neurons): {elapsed:.2f}ms')
print(f'   Average: {elapsed/100:.2f}ms per neuron')
"

echo ""
echo "================================================"
echo "✅ Build complete!"
echo ""
echo "Next steps:"
echo "1. Test performance: cd ../../ && pytest tests/bdu/ -v -k projector"
echo "2. Enable in Python: from feagi.bdu.connectivity.rust_bridge import enable_rust_synaptogenesis"
echo "3. Run real projection test to validate 40s → <1s improvement"
echo ""
echo "To rebuild: ./build_bdu.sh"

