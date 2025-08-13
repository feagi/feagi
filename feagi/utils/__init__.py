"""Utilities for FEAGI.

This package contains various utility functions and classes used throughout
FEAGI.
"""

from feagi.utils.config import FeagiConfig
from feagi.utils.logger import EmojiAdapter, StatusAdapter, setup_logger
from feagi.utils.version_checker import check_dependencies, verify_dependencies

__all__ = [
    "setup_logger",
    "StatusAdapter",
    "EmojiAdapter",  # Backward compatibility alias
    "check_dependencies",
    "verify_dependencies",
    "FeagiConfig",
]
