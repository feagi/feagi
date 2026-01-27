# Examples Virtual Environment Setup

This directory contains a dedicated virtual environment for running the next-generation FEAGI connector examples.

## Setup

The virtual environment has been pre-configured with all necessary dependencies:

### Core Dependencies
- `numpy>=1.20.0` - Array processing
- `aiohttp>=3.9.0` - Async HTTP client
- `toml>=0.10.2` - Configuration file parser
- `requests>=2.31.0` - HTTP requests

### Build Tools
- `maturin==1.8.3` - For building Rust Python extensions

### Development Tools
- `pytest>=7.0.0` - Testing framework

### Python Packages
- `feagi_rust_py_libs` - Rust-based performance libraries (installed from `/feagi-rust-py-libs`)
- `feagi_connector` - FEAGI connector package (installed in editable mode)
- `feagi_connector_2` - Next-generation FEAGI connector (included with feagi_connector)

## Activating the Virtual Environment

### macOS/Linux:
```bash
source example_venv/bin/activate
```

### Windows:
```bash
example_venv\Scripts\activate
```

## Running Examples

Once activated, you can run any example:

```bash
# Run the basic example
python example_2_2.py
```

## Deactivating

To exit the virtual environment:
```bash
deactivate
```

## Rebuilding Dependencies

If you need to rebuild the Rust libraries:

```bash
source example_venv/bin/activate
cd ../../feagi-rust-py-libs
maturin develop --release
```

## Notes

- The virtual environment is isolated and won't affect your system Python installation
- All packages are installed in development/editable mode where applicable
- Changes to the source code will be immediately reflected without reinstallation

