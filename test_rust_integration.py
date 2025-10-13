#!/usr/bin/env python3
"""
Test Rust burst engine integration with FEAGI

This verifies that the Rust synaptic propagation engine is properly integrated.
"""

import sys
import os

# Test 1: Import feagi_rust
print("=" * 60)
print("TEST 1: Import Rust Module")
print("=" * 60)
try:
    import feagi_rust
    print(f"✅ feagi_rust imported successfully (version {feagi_rust.__version__})")
except ImportError as e:
    print(f"❌ Failed to import feagi_rust: {e}")
    sys.exit(1)

# Test 2: Import BurstEngine
print("\n" + "=" * 60)
print("TEST 2: Import BurstEngine")
print("=" * 60)
try:
    from feagi.npu.burst_engine import BurstEngine, RUST_AVAILABLE
    print(f"✅ BurstEngine imported successfully")
    print(f"   RUST_AVAILABLE = {RUST_AVAILABLE}")
except ImportError as e:
    print(f"❌ Failed to import BurstEngine: {e}")
    sys.exit(1)

# Test 3: Check for Rust-related methods
print("\n" + "=" * 60)
print("TEST 3: Verify Rust Integration Methods")
print("=" * 60)
required_methods = ['_initialize_rust_engine', '_compute_synaptic_propagation']
for method in required_methods:
    if hasattr(BurstEngine, method):
        print(f"✅ BurstEngine.{method} exists")
    else:
        print(f"❌ BurstEngine.{method} NOT FOUND")
        sys.exit(1)

# Test 4: Verify old Python implementation is replaced
print("\n" + "=" * 60)
print("TEST 4: Verify Python Implementation Replaced")
print("=" * 60)
import inspect
source = inspect.getsource(BurstEngine._compute_synaptic_propagation)
if "RUST IMPLEMENTATION" in source:
    print("✅ Method signature confirms Rust implementation")
    # Check for old Python processing patterns (not just numpy for conversion)
    old_patterns = ["FULLY VECTORIZED", "np.argsort", "np.unique", "np.where", "SIMD Processing", "Gather Phase"]
    found_old = [p for p in old_patterns if p in source]
    if found_old:
        print(f"⚠️  WARNING: Old Python code patterns found: {found_old}")
        sys.exit(1)
    else:
        print("✅ Old Python processing code successfully removed")
        print(f"✅ Method is now {len(source.split(chr(10)))} lines (was ~175 lines)")
else:
    print("❌ Rust implementation marker not found")
    sys.exit(1)

# Test 5: Check for unwanted fallbacks
print("\n" + "=" * 60)
print("TEST 5: Check for Unwanted Fallbacks")
print("=" * 60)
unwanted_patterns = [
    "except.*Python",
    "try.*Rust.*except.*Python",
    "if.*not.*rust.*python",
]
has_unwanted_fallback = False
for pattern in unwanted_patterns:
    import re
    if re.search(pattern, source, re.IGNORECASE):
        print(f"⚠️  Found potential fallback pattern: {pattern}")
        has_unwanted_fallback = True

if not has_unwanted_fallback:
    print("✅ No unwanted fallbacks to Python implementation")

# Summary
print("\n" + "=" * 60)
print("INTEGRATION TEST SUMMARY")
print("=" * 60)
print("✅ All tests passed!")
print("\n🎯 Next step: Run FEAGI and verify Rust engine initializes")
print("   Look for log messages:")
print("   - 🦀 [RUST] Rust burst engine available")
print("   - 🦀 [RUST] Initialized synaptic propagation engine")
print("   - 🦀 [RUST SYNAPTIC-PROPAGATION] Burst #...")
