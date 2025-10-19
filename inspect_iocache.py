#!/usr/bin/env python3
"""
Inspect IOCache to see what methods are available
"""

import feagi_rust_py_libs as frpl

# Create IOCache instance
cache = frpl.connector_core.caching.IOCache()

print("=== IOCache Methods ===\n")
methods = [m for m in dir(cache) if not m.startswith('_')]
for method in sorted(methods):
    print(f"  {method}")

print("\n=== Checking for byte-related methods ===\n")
byte_methods = [m for m in methods if 'byte' in m.lower() or 'encode' in m.lower() or 'get' in m.lower()]
for method in byte_methods:
    print(f"  ✓ {method}")

print("\n=== Checking for sensor methods ===\n")
sensor_methods = [m for m in methods if 'sensor' in m.lower()]
for method in sensor_methods:
    print(f"  ✓ {method}")

