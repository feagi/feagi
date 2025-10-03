#!/usr/bin/env python3
"""
Test script to verify the rust_core_sensiomotor_functions_py module can be imported correctly.
"""
import sys
import os

# Add the directory containing the compiled module to Python's path
# Adjust this path as needed based on your build location
module_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "target/release"  # or "target/debug" if built in debug mode
)
sys.path.append(module_dir)

try:
    # Try to import the module
    import rust_core_sensiomotor_functions_py as rcsf
    print("✅ Successfully imported rust_core_sensiomotor_functions_py module")

    # Check if ImageDiff class is available
    if hasattr(rcsf, 'ImageDiff'):
        print("✅ ImageDiff class is available")
        
        # Try to instantiate an ImageDiff object
        image_diff = rcsf.ImageDiff(100, 100, 3)  # width, height, color_depth
        print(f"✅ Created ImageDiff instance with shape: {image_diff.shape()}")
    else:
        print("❌ ImageDiff class is not available in the module")
        print(f"Available attributes: {dir(rcsf)}")
        
except ImportError as e:
    print(f"❌ Failed to import rust_core_sensiomotor_functions_py: {e}")
    print("\nPossible issues and solutions:")
    print("1. The module may not be built yet. Run 'cargo build --release' in the rust_core_sensiomotor_functions_py directory.")
    print("2. The module may be named differently. Check if a file like 'rust_core_sensiomotor_functions_py.so' or 'rust_core_sensiomotor_functions_py.pyd' exists.")
    print("3. The module may be in a different location. Check the build output for the library location.")
    
    # Try to list available .so or .pyd files in the expected location
    if os.path.exists(module_dir):
        files = [f for f in os.listdir(module_dir) if f.endswith('.so') or f.endswith('.pyd')]
        if files:
            print(f"\nFound these potential module files in {module_dir}:")
            for f in files:
                print(f"  - {f}")
        else:
            print(f"\nNo .so or .pyd files found in {module_dir}")
    else:
        print(f"\nDirectory {module_dir} does not exist") 