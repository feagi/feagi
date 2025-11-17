"""
Plasticity package for FEAGI NPU

Plasticity functionality has been migrated to Rust for performance.
All plasticity computations (STDP, memory formation, pattern detection)
are now implemented in the feagi-plasticity Rust crate.

This module remains as a placeholder for backward compatibility.
The actual implementation is accessed through the Rust NPU bindings.
"""

# Note: Plasticity is now implemented in Rust
# Access through: feagi_rust.plasticity or the NPU's built-in plasticity system

__all__ = []
