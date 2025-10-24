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
    from feagi_rust_py_libs.feagi_python import *  # noqa: F401, F403
    RUST_AVAILABLE = True
except ImportError as e:
    RUST_AVAILABLE = False
    warnings.warn(
        f"Failed to import Rust extensions from feagi-rust-py-libs: {e}\n"
        "Install with: pip install feagi-rust-py-libs\n"
        "Some high-performance functionality may be unavailable."
    )

# Re-export for backward compatibility
__all__ = ["RUST_AVAILABLE"]
