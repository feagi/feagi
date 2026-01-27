#!/bin/bash
# Quick activation script for the examples virtual environment
# Usage: source activate.sh

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/example_venv/bin/activate"

echo "✓ Virtual environment activated"
echo "  Python: $(which python)"
echo "  Location: $VIRTUAL_ENV"
echo ""
echo "Available packages:"
echo "  - feagi_connector_2 (next-gen connector)"
echo "  - feagi_rust_py_libs (Rust performance libs)"
echo "  - numpy, aiohttp, toml"
echo ""
echo "To deactivate, run: deactivate"

