"""
Rust integration for FEAGI.

MIGRATION NOTE: Rust code has been moved to separate packages:
- Pure Rust libraries → feagi-core repository
- PyO3 Python bindings → feagi-rust-py-libs package

This module serves as a compatibility shim for backward compatibility.
"""

import warnings

try:
    # Import from new feagi-rust-py-libs package
    # Note: feagi_python is a submodule added via m.add_submodule() in lib.rs
    import feagi_rust_py_libs
    feagi_python = feagi_rust_py_libs.feagi_python
    # Re-export all public items for backward compatibility
    RustNPU = feagi_python.RustNPU
    BurstResult = feagi_python.BurstResult
    PyPNS = feagi_python.PyPNS
    PyAgentRegistry = feagi_python.PyAgentRegistry
    VisualizationEncoder = feagi_python.VisualizationEncoder
    FeagiByteStructure = feagi_python.FeagiByteStructure
    CorticalMappedXYZPNeuronDataDecoder = feagi_python.CorticalMappedXYZPNeuronDataDecoder
    export_connectome_bytes = feagi_python.export_connectome_bytes
    import_connectome_bytes = feagi_python.import_connectome_bytes
    save_connectome_to_file = feagi_python.save_connectome_to_file
    load_connectome_from_file = feagi_python.load_connectome_from_file
    test_simple_function = feagi_python.test_simple_function
    RUST_AVAILABLE = True
except (ImportError, AttributeError) as e:
    # NO FALLBACKS - FAIL FAST
    raise RuntimeError(
        f"🦀 [RUST] CRITICAL: Failed to import Rust extensions from feagi-rust-py-libs: {e}\n"
        "FEAGI REQUIRES Rust components to run. Install with:\n"
        "  cd /path/to/feagi-rust-py-libs\n"
        "  cargo build --release\n"
        "  cp target/release/libfeagi_rust_py_libs.dylib feagi_data_processing.so\n"
        "  (add to PYTHONPATH or install package)\n"
        "\n"
        "FEAGI WILL NOT RUN WITHOUT RUST COMPONENTS - NO FALLBACKS ALLOWED"
    ) from e

# Re-export for backward compatibility
__all__ = ["RUST_AVAILABLE"]
