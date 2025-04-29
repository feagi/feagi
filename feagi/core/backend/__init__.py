"""
Backend Abstraction Layer for FEAGI.

This module provides a unified interface for different computational backends,
enabling FEAGI to run on various hardware configurations including CPU and GPU.
"""

from feagi.core.backend.interface import (
    BackendType,
    BackendInterface,
    get_available_backends,
    get_backend,
)

# Import backend implementations
from feagi.core.backend.cpu import CPUBackend

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