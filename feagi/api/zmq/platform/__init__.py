"""Platform-specific optimizations for neural data transmission.

This package provides platform-aware socket configuration to maximize
performance across different operating systems.
"""

from .optimizer import (
    PlatformOptimizer,
    get_platform_info,
    get_platform_optimizer,
    optimize_socket_for_neural_data,
)

__all__ = [
    "PlatformOptimizer",
    "get_platform_optimizer",
    "optimize_socket_for_neural_data",
    "get_platform_info",
]
