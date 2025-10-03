#!/bin/bash
# Build and test the Rust library for Python

set -e  # Exit on any error

echo "Building Rust library for Python..."
cd rust_core_sensiomotor_functions_py
cargo build --release

echo -e "\nCopying library to a location Python can find..."
# The exact output filename depends on the OS
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    LIB_EXT=".dylib"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    # Windows
    LIB_EXT=".dll"
else
    # Linux or other Unix-like
    LIB_EXT=".so"
fi

# Find the compiled library
LIB_PATH=$(find target/release -name "*$LIB_EXT" -o -name "*.pyd" | grep -v '\.d')

if [ -z "$LIB_PATH" ]; then
    echo "Error: Could not find compiled library"
    exit 1
fi

echo "Found library at: $LIB_PATH"

# Extract just the filename
LIB_FILENAME=$(basename "$LIB_PATH")

# Verify the module name
echo -e "\nVerifying module name..."
if [[ "$LIB_FILENAME" != "FEAGI_Connector"* ]]; then
    echo "Warning: Library filename ($LIB_FILENAME) does not match expected module name (FEAGI_Connector)"
    echo "This might prevent Python from importing it correctly"
    
    # See what the actual module name might be
    POTENTIAL_MODULE_NAME="${LIB_FILENAME%%.*}"
    echo "Python might try to import it as '$POTENTIAL_MODULE_NAME' instead of 'FEAGI_Connector'"
fi

cd ..
echo -e "\nRunning test script..."
python test_import.py 