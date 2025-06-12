"""
Backend Abstraction Layer for FEAGI.

This module provides a unified interface for different computational backends,
enabling FEAGI to run on various hardware configurations including CPU and GPU.
"""

# Import backend implementations
from feagi.core.backend.cpu import CPUBackend
from feagi.core.backend.interface import (
    BackendInterface,
    BackendType,
    get_available_backends,
    get_backend,
)

try:
    from feagi.core.backend.webgpu import WebGPUBackend
except ImportError:
    # WebGPU support is optional
    pass

__all__ = [
    "BackendType",
    "BackendInterface",
    "get_available_backends",
    "get_backend",
    "CPUBackend",
    "WebGPUBackend",
]
