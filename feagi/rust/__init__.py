"""Rust integration for FEAGI."""

# This module will contain bindings to Rust code
# The actual Rust code will be compiled during package installation

try:
    from feagi.rust.feagi_rust import *  # type: ignore
except ImportError:
    import warnings
    warnings.warn(
        "Failed to import Rust extensions. Some functionality may be unavailable. "
        "This could be due to missing Rust compiler or failed compilation during installation."
    ) 