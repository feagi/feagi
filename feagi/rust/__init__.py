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
    RUST_AVAILABLE = False
    warnings.warn(
        f"Failed to import Rust extensions from feagi-rust-py-libs: {e}\n"
        "Install with: pip install feagi-rust-py-libs\n"
        "Some high-performance functionality may be unavailable."
    )

# Re-export for backward compatibility
__all__ = ["RUST_AVAILABLE"]
