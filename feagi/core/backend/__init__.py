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

__all__ = [
    "BackendType",
    "BackendInterface",
    "get_available_backends",
    "get_backend",
] 