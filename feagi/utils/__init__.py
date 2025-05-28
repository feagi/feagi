"""Utilities for FEAGI.

This package contains various utility functions and classes used throughout FEAGI."""

from feagi.utils.logger import setup_logger, StatusAdapter, EmojiAdapter
from feagi.utils.version_checker import check_dependencies, verify_dependencies
from feagi.utils.config import FeagiConfig

__all__ = [
    "setup_logger", 
    "StatusAdapter", 
    "EmojiAdapter",  # Backward compatibility alias
    "check_dependencies", 
    "verify_dependencies", 
    "FeagiConfig"
] 