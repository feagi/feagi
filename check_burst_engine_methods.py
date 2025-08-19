#!/usr/bin/env python3
import sys
sys.path.insert(0, '/Users/nadji/code/FEAGI-2.0/feagi_core')

from feagi.npu.burst_engine_npu_integration import patch_burst_engine_for_npu
from feagi.npu.burst_engine import BurstEngine

print("Before patch:")
methods_before = [m for m in dir(BurstEngine) if 'npu' in m.lower() or 'burst' in m.lower()]
print(f"NPU/burst methods: {methods_before}")

patch_burst_engine_for_npu()

print("\nAfter patch:")
methods_after = [m for m in dir(BurstEngine) if 'npu' in m.lower() or 'burst' in m.lower()]
print(f"NPU/burst methods: {methods_after}")

print(f"\nAll methods containing 'process':")
process_methods = [m for m in dir(BurstEngine) if 'process' in m.lower()]
print(f"Process methods: {process_methods}")
