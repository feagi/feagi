"""
NPU Services for Core API

This module provides NPU-specific services for the Core API, exposing
the new NPU interface operations through REST and ZMQ endpoints.
"""

from .npu_service import NPUService

__all__ = [
    'NPUService'
]
